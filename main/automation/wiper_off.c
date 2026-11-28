#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "wiper_off";

#define BUS  1

#define DAS_BODY_CONTROLS_ID 0x3E9
#define SCCM_LEFT_STALK_ID   0x249   // the steering column module sends it every 50 ms
#define VCLEFT_SWITCH_ID     0x3C2   // the left body controller sends it every 50 ms

// The sequence transmits under two ids the car's own modules also transmit.
// Two nodes that start the same id at the same moment collide, and neither
// backs off. On a car, one unsynchronised sequence coincided with the steering
// column module going silent for a second (steering angle missing, "Lane
// Departure Avoidance features unavailable"). So each of our frames goes out
// right after the car's own frame of that id, in the part of its period where
// the module has nothing to send.
#define SAFE_AFTER_CAR_US    30000   // of the 50 ms period; the rest is margin for a late or early frame
#define CAR_FRAME_WAIT_MS    70      // no frame for this long: nothing to follow, so nothing is sent

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

// ---- Following the car's own frames (see SAFE_AFTER_CAR_US) ----
static SemaphoreHandle_t s_car_frame_sem;        // given on each car frame of s_follow_id
static volatile uint32_t s_follow_id;            // 0: not following
static volatile uint32_t s_car_frame_us;         // when the last one arrived (low 32 bits of the timer)

// True while a frame sent now is clear of the car's next frame of the followed id.
static bool clear_of_car_frame(void)
{
    return (uint32_t)esp_timer_get_time() - s_car_frame_us < SAFE_AFTER_CAR_US;
}

// Wait for a fresh frame of `id` from the car. False: none came (the module is
// silent, or our receive is behind), and then nothing may be sent under that id.
static bool wait_car_frame(uint32_t id)
{
    s_follow_id = id;
    return xSemaphoreTake(s_car_frame_sem, pdMS_TO_TICKS(CAR_FRAME_WAIT_MS)) == pdTRUE
           && clear_of_car_frame();
}

// ---- Wiper-off sequence (transient self-deleting task: blocking multi-step) ----
static volatile bool s_seq_running;

static void wiper_off_seq_task(void *arg)
{
    (void)arg;
    int sent = 0, missed = 0;
    ESP_LOGI(TAG, "sequence starting");
    xSemaphoreTake(s_car_frame_sem, 0);   // drop a stale give

    // Phase 1: hold the wiper button (wash/wipe = 1ST_DETENT) for 300 ms: after
    // each of the car's own SCCM_leftStalk frames, ours at 100 Hz for as long
    // as they stay clear of its next one (three frames).
    int64_t end = esp_timer_get_time() + 300000;
    while (esp_timer_get_time() < end) {
        if (!wait_car_frame(SCCM_LEFT_STALK_ID)) {
            missed++;
            continue;
        }
        // Continue the car's rolling counter (next send emits last_seen + 1).
        dbc_counter_seed_from_bus(BUS, "SCCM_leftStalk");
        do {
            can_send(BUS, "SCCM_leftStalk", "SCCM_washWipeButtonStatus", 1, false);
            sent++;
            vTaskDelay(pdMS_TO_TICKS(10));
        } while (clear_of_car_frame() && esp_timer_get_time() < end);
    }

    // Gap: let the car register the button hold.
    s_follow_id = 0;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Phase 2: scroll-down tick (swcLeftScrollTicks = -1, auto-muxed). Twice for
    // reliability, each right after one of the car's own VCLEFT_switchStatus frames.
    xSemaphoreTake(s_car_frame_sem, 0);   // nothing is followed during the gap, so this empties it for good
    for (int i = 0; i < 2; i++) {
        if (!wait_car_frame(VCLEFT_SWITCH_ID)) {
            missed++;
            continue;
        }
        can_send(BUS, "VCLEFT_switchStatus", "VCLEFT_swcLeftScrollTicks", -1, false);
        sent++;
    }
    s_follow_id = 0;
    ESP_LOGI(TAG, "sequence done: %d frames sent, %d waits without a car frame to follow", sent, missed);
    s_seq_running = false;

    vTaskDelete(NULL);
}

// ---- Automation hooks ----
static void wiper_off_init(automation_t *self)
{
    (void)self;
    load_enabled();
    if (!s_car_frame_sem) s_car_frame_sem = xSemaphoreCreateBinary();
    s_armed = false;
    ESP_LOGI(TAG, "auto wiper-off ready (%s)", s_enabled ? "enabled" : "disabled");
}

static void wiper_off_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;

    // The running sequence follows the car's own frames of one id.
    if (s_follow_id && frame->bus_id == BUS && frame->frame.id == s_follow_id) {
        s_car_frame_us = (uint32_t)esp_timer_get_time();
        xSemaphoreGive(s_car_frame_sem);
        return;
    }

    // Called for every received frame; only a fresh DAS_bodyControls can
    // change the wiper speed (can_get reads the cached copy by name).
    if (frame->frame.id != DAS_BODY_CONTROLS_ID) {
        return;
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
        // ADAS on: TACC engaged (DAS_accState ACC_HOLD 3 / ACC_ON 4; with
        // openpilot the car only ever sees TACC) or Autosteer engaged
        // (DAS_autopilotState ACTIVE_NOMINAL/RESTRICTED/NAV 3-5). DAS_lssState
        // is the lane safety system (LKA/ELK interventions, 7 = OFF), not this.
        double acc, ap;
        bool tacc = can_get(0, "DAS_control", "DAS_accState", &acc, false) == ESP_OK
                    && (acc == 3.0 || acc == 4.0);
        bool autosteer = can_get(0, "DAS_status", "DAS_autopilotState", &ap, false) == ESP_OK
                         && ap >= 3.0 && ap <= 5.0;
        bool adas_on = tacc || autosteer;
        if (s_enabled && adas_on && s_car_frame_sem && !s_seq_running) {
            ESP_LOGW(TAG, "DAS_wiperSpeed OFF->AUTO with ADAS on, triggering");
            s_seq_running = true;
            if (xTaskCreatePinnedToCore(wiper_off_seq_task, "wiper_off", 4096, NULL, 5, NULL, 0) != pdPASS) {
                s_seq_running = false;
            }
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
