#include "three_finger.h"
#include "vehicle_control.h"
#include "battery_preheat.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

static const char *TAG = "three_finger";

// Persist the bound action so it survives a reboot/power-cycle even before the
// Android app reconnects and re-syncs it.
#define NVS_NAMESPACE  "three_finger"
#define NVS_KEY_ACTION "action"

#define BUS                 1
// UI_status2 (991 / 0x3DF, 8 bytes). UI_activeTouchPoints is an 8-bit Intel
// signal at start bit 24, i.e. data byte 3: the number of fingers currently
// touching the infotainment screen.
#define UI_STATUS2_ID       0x3DF
#define TOUCH_POINTS_BYTE   3
#define THREE_FINGERS       3

// After firing, ignore further taps for this long.
#define REFIRE_LOCKOUT_US   (1500 * 1000)

static volatile uint8_t s_action = THREE_FINGER_ACTION_NONE;
static volatile bool s_mirrors_folded = false;
static volatile bool    s_armed = true;
static volatile int64_t s_last_fire_us = 0;
static volatile uint8_t s_last_logged_points = 0xFF;

static void save_action(uint8_t action)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(nvs, NVS_KEY_ACTION, action);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist action: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static uint8_t load_action(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return THREE_FINGER_ACTION_NONE;
    }
    uint8_t action = THREE_FINGER_ACTION_NONE;
    nvs_get_u8(nvs, NVS_KEY_ACTION, &action);
    nvs_close(nvs);
    return action;
}

void three_finger_set_action(uint8_t action)
{
    if (action != s_action) {
        ESP_LOGW(TAG, "three-finger action set to %u", action);
        s_action = action;
        save_action(action);
        return;
    }
    s_action = action;
}

static void fire_action(uint8_t action)
{
    switch (action) {
    case THREE_FINGER_ACTION_GLOVEBOX:
        ESP_LOGI(TAG, "3-finger tap -> glovebox");
        vehicle_control_submit(VC_CMD_GLOVEBOX, 1);
        break;
    case THREE_FINGER_ACTION_PREHEAT:
        ESP_LOGI(TAG, "3-finger tap -> battery preheat toggle");
        vehicle_control_submit(VC_CMD_BATTERY_PREHEAT,
                               battery_preheat_enabled() ? 0 : 1);
        break;
    case THREE_FINGER_ACTION_MIRROR_FOLD:
        s_mirrors_folded = !s_mirrors_folded;
        ESP_LOGI(TAG, "3-finger tap -> mirrors %s",
                 s_mirrors_folded ? "fold" : "unfold");
        vehicle_control_submit(VC_CMD_MIRROR_FOLD, s_mirrors_folded ? 1 : 2);
        break;
    default:
        break;
    }
}

void three_finger_observe(const can_tagged_frame_t *frame)
{
    if (!frame || frame->bus_id != BUS) return;
    if (frame->frame.id != UI_STATUS2_ID) return;
    if (frame->frame.dlc <= TOUCH_POINTS_BYTE) return;

    uint8_t points = frame->frame.data[TOUCH_POINTS_BYTE];

    // Log every change in the touch count so the gesture can be traced live.
    if (points != s_last_logged_points) {
        ESP_LOGI(TAG, "touchPoints %u->%u (armed=%d action=%u)",
                 s_last_logged_points, points, (int)s_armed, s_action);
        s_last_logged_points = points;
    }

    // Re-arm only once every finger has lifted
    if (points == 0) {
        if (!s_armed) {
            ESP_LOGI(TAG, "all fingers lifted, re-armed");
        }
        s_armed = true;
        return;
    }

    if (!s_armed || points < THREE_FINGERS ||
        s_action == THREE_FINGER_ACTION_NONE) {
        return;
    }

    int64_t now = esp_timer_get_time();
    if (now - s_last_fire_us < REFIRE_LOCKOUT_US) {
        ESP_LOGW(TAG, "3-finger tap ignored: in %lld ms lockout",
                 (long long)((REFIRE_LOCKOUT_US - (now - s_last_fire_us)) / 1000));
        s_armed = false;  // swallow this gesture; still wait for a release
        return;
    }

    s_armed = false;
    s_last_fire_us = now;
    ESP_LOGW(TAG, "3-finger tap detected (points=%u) -> firing action %u",
             points, s_action);
    fire_action(s_action);
}

esp_err_t three_finger_init(void)
{
    s_action = load_action();
    s_armed = true;
    s_last_fire_us = 0;
    s_last_logged_points = 0xFF;
    ESP_LOGI(TAG, "three-finger automation initialized (bus %d id 0x%03X, action %u)",
             BUS, UI_STATUS2_ID, s_action);
    return ESP_OK;
}
