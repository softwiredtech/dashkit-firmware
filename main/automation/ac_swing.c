// AC swing: sweep the driver vent left<->right by RMW of the car's live
// UI_ventPanelControlRequest mux-0 frame (byte1 = driver X, raw 0..200).

#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "ac_swing";

#define VENT_BUS      1
#define VENT_MSG      "UI_ventPanelControlRequest"
#define VENT_ID       0x253
#define VENT_MUX_POS  0
#define SIG_LEFT_X    "UI_ventPanelLeftPositionX"

#define X_MIN_RAW     0      // left
#define X_MAX_RAW     200    // right
#define SWEEP_MS      8000   // one direction
#define INJECT_MS     40     // one raw step per frame; car idles at 2Hz
#define BASE_STALE_US (5 * 1000 * 1000)

#define NVS_NAMESPACE   "ac_swing"
#define NVS_KEY_ENABLED "en"

static volatile bool s_enabled = false;
static bool          s_active;

static portMUX_TYPE s_base_lock = portMUX_INITIALIZER_UNLOCKED;
static can_frame_t  s_base;
static bool         s_have_base;
static int64_t      s_base_us;
static dbc_sig_t    s_sig_left_x;

static void inject_task(void *arg);

static void save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(nvs, NVS_KEY_ENABLED, s_enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist config: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static void load_config(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    uint8_t v = 0;
    if (nvs_get_u8(nvs, NVS_KEY_ENABLED, &v) == ESP_OK) {
        s_enabled = (v != 0);
    }
    nvs_close(nvs);
}

static void ac_swing_init(automation_t *self)
{
    (void)self;
    load_config();
    s_sig_left_x = dbc_sig(dbc_msg(VENT_MSG), SIG_LEFT_X);
    xTaskCreatePinnedToCore(inject_task, "ac_swing_inj", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "AC swing READY (%s), %dms per sweep @ %dms",
             s_enabled ? "ENABLED" : "disabled", SWEEP_MS, INJECT_MS);
}

static void ac_swing_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;
    if (frame->bus_id != VENT_BUS || frame->frame.id != VENT_ID
        || frame->frame.data[0] != VENT_MUX_POS) {
        return;
    }
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_base_lock);
    s_base = frame->frame;
    s_have_base = true;
    s_base_us = now;
    portEXIT_CRITICAL(&s_base_lock);
}

static uint8_t swing_x(int64_t now_us)
{
    int64_t t = (now_us / 1000) % (2 * SWEEP_MS);
    if (t >= SWEEP_MS) {
        t = 2 * SWEEP_MS - t;
    }
    return (uint8_t)(X_MIN_RAW + (X_MAX_RAW - X_MIN_RAW) * t / SWEEP_MS);
}

static void set_active(bool active, const char *why)
{
    if (active == s_active) {
        return;
    }
    s_active = active;
    ESP_LOGW(TAG, "%s swing (%s)", active ? "START" : "STOP", why);
}

static void inject_task(void *arg)
{
    (void)arg;
    while (true) {
        const char *why = NULL;
        can_frame_t f;
        int64_t now = esp_timer_get_time();

        if (!s_enabled) {
            why = "disabled";
        } else {
            bool fresh = false;
            portENTER_CRITICAL(&s_base_lock);
            if (s_have_base) {
                f = s_base;
                fresh = (now - s_base_us) < BASE_STALE_US;
            }
            portEXIT_CRITICAL(&s_base_lock);
            if (!fresh) {
                why = "no fresh vent frame";
            }
        }

        if (why) {
            set_active(false, why);
        } else {
            set_active(true, "enabled, live vent frame");
            dbc_pack(f.data, s_sig_left_x, swing_x(now));
            can_frame_send(VENT_BUS, VENT_MSG, &f);
        }
        vTaskDelay(pdMS_TO_TICKS(INJECT_MS));
    }
}

static void ac_swing_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode != VC_CMD_AC_SWING_ENABLE) {
        return;
    }
    bool enabled = (value != 0);
    if (enabled != s_enabled) {
        ESP_LOGW(TAG, "automation %s", enabled ? "ENABLED" : "disabled");
        s_enabled = enabled;
        save_config();
    }
}

automation_t ac_swing_automation = {
    .name           = "ac_swing",
    .tick_period_ms = 0,
    .init           = ac_swing_init,
    .on_frame       = ac_swing_on_frame,
    .on_config      = ac_swing_on_config,
};
