#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "wiper_off";

#define BUS  1

// Juniper: the wiper button lives in VCLEFT_switchStatus mux 1 (no checksum).
#define SWC_MSG          "VCLEFT_switchStatus"
#define SWC_ID           0x3C2
#define SWC_MUX          1
#define WIPER_HARD_PRESS 2
#define SWC_SNAPSHOT_US  (2 * 1000 * 1000)

static portMUX_TYPE s_swc_lock = portMUX_INITIALIZER_UNLOCKED;
static can_frame_t  s_swc;
static int64_t      s_swc_us;
static bool         s_have_swc;

// Persist the enabled flag so it survives a reboot/power-cycle even before the
// Android app reconnects and re-syncs it.
#define NVS_NAMESPACE   "wiper_off"
#define NVS_KEY_ENABLED "en"

static volatile bool s_armed = false;
// Off by default (matches the Android app default); enabled over BLE.
static volatile bool s_enabled = false;

static void save_enabled(bool enabled)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(nvs, NVS_KEY_ENABLED, enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist enabled flag: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static void load_enabled(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;  // keep default (disabled)
    }
    uint8_t enabled = 0;
    if (nvs_get_u8(nvs, NVS_KEY_ENABLED, &enabled) == ESP_OK) {
        s_enabled = (enabled != 0);
    }
    nvs_close(nvs);
}

// ---- Wiper-off sequence (transient self-deleting task: blocking multi-step) ----
static void wiper_off_seq_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "sequence starting");

    dbc_msg_t m = dbc_msg(SWC_MSG);
    dbc_sig_t sig_mux    = dbc_sig(m, "VCLEFT_switchStatusIndex");
    dbc_sig_t sig_button = dbc_sig(m, "VCLEFT_swcWiperButtonState");
    dbc_sig_t sig_scroll = dbc_sig(m, "VCLEFT_swcLeftScrollTicks");

    // Base every injected frame on the car's latest mux-1 frame so the other
    // switch fields stay valid; a zeroed frame would report them all as SNA.
    can_frame_t base;
    bool have;
    portENTER_CRITICAL(&s_swc_lock);
    have = s_have_swc && esp_timer_get_time() - s_swc_us < SWC_SNAPSHOT_US;
    base = s_swc;
    portEXIT_CRITICAL(&s_swc_lock);
    if (!have) {
        ESP_LOGW(TAG, "no fresh %s mux-1 frame, sending from scratch", SWC_MSG);
        can_frame_init(SWC_MSG, &base);
        dbc_pack(base.data, sig_mux, SWC_MUX);
    }
    dbc_pack(base.data, sig_button, 0);
    dbc_pack(base.data, sig_scroll, 0);

    // Phase 1: hard-press the wiper button for 200ms at 100Hz (the car's own
    // press lasted 180ms), then release. This opens the wiper menu.
    can_frame_t f = base;
    dbc_pack(f.data, sig_button, WIPER_HARD_PRESS);
    int64_t end = esp_timer_get_time() + 200000;
    while (esp_timer_get_time() < end) {
        can_frame_send(BUS, SWC_MSG, &f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (int i = 0; i < 3; i++) {
        f = base;
        can_frame_send(BUS, SWC_MSG, &f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Gap: the menu opened ~90ms after the press on the recorded drive.
    vTaskDelay(pdMS_TO_TICKS(500));

    // Phase 2: scroll-down tick (AUTO -> OFF). Twice for reliability.
    for (int i = 0; i < 2; i++) {
        f = base;
        dbc_pack(f.data, sig_scroll, -1);
        can_frame_send(BUS, SWC_MSG, &f);
        if (i == 0) vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "sequence done (live base=%d)", have);

    vTaskDelete(NULL);
}

// ---- Automation hooks ----
static void wiper_off_init(automation_t *self)
{
    (void)self;
    load_enabled();
    s_armed = false;
    ESP_LOGI(TAG, "auto wiper-off ready (%s)", s_enabled ? "enabled" : "disabled");
}

static void wiper_off_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;

    if (frame->bus_id == BUS && frame->frame.id == SWC_ID && (frame->frame.data[0] & 0x3) == SWC_MUX) {
        portENTER_CRITICAL(&s_swc_lock);
        s_swc = frame->frame;
        s_swc_us = esp_timer_get_time();
        s_have_swc = true;
        portEXIT_CRITICAL(&s_swc_lock);
    }

    // DAS_wiperSpeed: 0 = AUTO, 1..14 = manual speeds, 15 = OFF.
    double speed;
    if (can_get(frame->bus_id, "DAS_bodyControls", "DAS_wiperSpeed", &speed, false) != ESP_OK) {
        return;
    }
    int s = (int)speed;

    if (s == 15) {
        s_armed = true;
        return;
    }

    if (s > 0) {
        s_armed = false;
        return;
    }

    if (s_armed) {
        s_armed = false;
        double lss;
        bool adas_on = can_get(0, "DAS_status2", "DAS_lssState", &lss, false) == ESP_OK
                       && lss >= 2.0;
        if (s_enabled && adas_on) {
            ESP_LOGW(TAG, "DAS_wiperSpeed OFF->AUTO with ADAS on, triggering");
            xTaskCreatePinnedToCore(wiper_off_seq_task, "wiper_off", 4096, NULL, 5, NULL, 0);
        } else {
            ESP_LOGI(TAG, "DAS_wiperSpeed OFF->AUTO, skipping (enabled=%d adas=%d)",
                     s_enabled, adas_on);
        }
    }
}

static void wiper_off_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode != VC_CMD_WIPER_OFF_ENABLE) {
        return;
    }
    bool enabled = (value != 0);
    if (enabled != s_enabled) {
        ESP_LOGW(TAG, "automation %s", enabled ? "enabled" : "disabled");
        s_enabled = enabled;
        save_enabled(enabled);
    }
}

automation_t wiper_off_automation = {
    .name           = "wiper_off",
    .tick_period_ms = 0,
    .init           = wiper_off_init,
    .on_frame       = wiper_off_on_frame,
    .on_config      = wiper_off_on_config,
};
