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

    // Align our rolling counter with the car's live SCCM_leftStalk so the
    // injected frames continue the sequence (next send emits last_seen + 1).
    dbc_counter_seed_from_bus(BUS, "SCCM_leftStalk");

    // Phase 1: hold the wiper button (wash/wipe = 1ST_DETENT) for 300ms at 100Hz.
    int64_t end = esp_timer_get_time() + 300000;
    while (esp_timer_get_time() < end) {
        can_send(BUS, "SCCM_leftStalk", "SCCM_washWipeButtonStatus", 1, false);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Gap: let the car register the button hold.
    vTaskDelay(pdMS_TO_TICKS(100));

    // Phase 2: scroll-down tick (swcLeftScrollTicks = -1, auto-muxed). Twice for
    // reliability.
    for (int i = 0; i < 2; i++) {
        can_send(BUS, "VCLEFT_switchStatus", "VCLEFT_swcLeftScrollTicks", -1, false);
        if (i == 0) vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "sequence done");

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
