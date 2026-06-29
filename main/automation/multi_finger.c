// Multi-finger infotainment gesture automation. Binds a 3/4/5-finger tap on the
// touchscreen (UI_status2.UI_activeTouchPoints) to a vehicle-control action.
//
// Touch reads now go through the DBC engine (can_get); the action itself is
// still fired via vehicle_control_submit so it shares the BLE command path.

#include "automation.h"
#include "multi_finger.h"
#include "vehicle_control.h"
#include "battery_preheat.h"
#include "dbc.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include <stdio.h>

static const char *TAG = "multi_finger";

#define BUS  1

// Persist the bound actions so they survive a reboot/power-cycle even before the
// Android app reconnects and re-syncs them. One u8 key per finger count.
#define NVS_NAMESPACE   "multi_finger"
#define NVS_KEY_FMT     "fp%u"   // -> "fp3", "fp4", "fp5"

// Number of bindable finger counts (3, 4, 5).
#define ACTION_SLOTS    (MULTI_FINGER_MAX_FINGERS - MULTI_FINGER_MIN_FINGERS + 1)

// After firing, ignore further taps for this long.
#define REFIRE_LOCKOUT_US   (1500 * 1000)

// Action bound to each finger count, indexed by (fingers - MULTI_FINGER_MIN_FINGERS).
static volatile uint8_t s_actions[ACTION_SLOTS] = { MULTI_FINGER_ACTION_NONE };
static volatile bool     s_mirrors_folded = false;
static volatile bool     s_charge_port_open = false;
// Peak touch-point count seen during the current gesture; the action is fired on
// release based on this peak so 3/4/5-finger taps can be told apart (fingers land
// one at a time, passing through the lower counts).
static volatile uint8_t  s_max_points = 0;
static volatile int64_t  s_last_fire_us = 0;
static volatile uint8_t  s_last_logged_points = 0xFF;

static void nvs_key_for(uint8_t fingers, char *out, size_t out_len)
{
    snprintf(out, out_len, NVS_KEY_FMT, fingers);
}

static void save_action(uint8_t fingers, uint8_t action)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    char key[8];
    nvs_key_for(fingers, key, sizeof(key));
    err = nvs_set_u8(nvs, key, action);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist %s: %s", key, esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static void load_actions(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }
    for (uint8_t fingers = MULTI_FINGER_MIN_FINGERS;
         fingers <= MULTI_FINGER_MAX_FINGERS; fingers++) {
        char key[8];
        nvs_key_for(fingers, key, sizeof(key));
        uint8_t action = MULTI_FINGER_ACTION_NONE;
        nvs_get_u8(nvs, key, &action);
        s_actions[fingers - MULTI_FINGER_MIN_FINGERS] = action;
    }
    nvs_close(nvs);
}

static void set_action(uint8_t fingers, uint8_t action)
{
    if (fingers < MULTI_FINGER_MIN_FINGERS || fingers > MULTI_FINGER_MAX_FINGERS) {
        ESP_LOGW(TAG, "ignoring action for unsupported finger count %u", fingers);
        return;
    }
    uint8_t idx = fingers - MULTI_FINGER_MIN_FINGERS;
    if (action != s_actions[idx]) {
        ESP_LOGW(TAG, "%u-finger action set to %u", fingers, action);
        s_actions[idx] = action;
        save_action(fingers, action);
    }
}

static void fire_action(uint8_t fingers, uint8_t action)
{
    switch (action) {
    case MULTI_FINGER_ACTION_GLOVEBOX:
        ESP_LOGI(TAG, "%u-finger tap -> glovebox", fingers);
        vehicle_control_submit(VC_CMD_GLOVEBOX, 1);
        break;
    case MULTI_FINGER_ACTION_PREHEAT:
        ESP_LOGI(TAG, "%u-finger tap -> battery preheat toggle", fingers);
        vehicle_control_submit(VC_CMD_BATTERY_PREHEAT,
                               battery_preheat_enabled() ? 0 : 1);
        break;
    case MULTI_FINGER_ACTION_MIRROR_FOLD:
        s_mirrors_folded = !s_mirrors_folded;
        ESP_LOGI(TAG, "%u-finger tap -> mirrors %s", fingers,
                 s_mirrors_folded ? "fold" : "unfold");
        vehicle_control_submit(VC_CMD_MIRROR_FOLD, s_mirrors_folded ? 1 : 2);
        break;
    case MULTI_FINGER_ACTION_FRUNK:
        ESP_LOGI(TAG, "%u-finger tap -> frunk", fingers);
        vehicle_control_submit(VC_CMD_CLOSURE, 2);  // FRONT_TRUNK
        break;
    case MULTI_FINGER_ACTION_TRUNK:
        ESP_LOGI(TAG, "%u-finger tap -> trunk", fingers);
        vehicle_control_submit(VC_CMD_CLOSURE, 1);  // REAR_TRUNK
        break;
    case MULTI_FINGER_ACTION_CHARGE_PORT:
        s_charge_port_open = !s_charge_port_open;
        ESP_LOGI(TAG, "%u-finger tap -> charge port %s", fingers,
                 s_charge_port_open ? "open" : "close");
        vehicle_control_submit(s_charge_port_open ? VC_CMD_CHARGE_PORT_OPEN
                                                  : VC_CMD_CHARGE_PORT_CLOSE, 1);
        break;
    default:
        break;
    }
}

// ---- Automation hooks ----
static void multi_finger_init(automation_t *self)
{
    (void)self;
    for (int i = 0; i < ACTION_SLOTS; i++) {
        s_actions[i] = MULTI_FINGER_ACTION_NONE;
    }
    load_actions();
    s_max_points = 0;
    s_last_fire_us = 0;
    s_last_logged_points = 0xFF;
    ESP_LOGI(TAG, "multi-finger ready (actions 3:%u 4:%u 5:%u)",
             s_actions[0], s_actions[1], s_actions[2]);
}

static void multi_finger_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;

    double points_d;
    if (can_get(frame->bus_id, "UI_status2", "UI_activeTouchPoints", &points_d, false) != ESP_OK) {
        return;
    }
    uint8_t points = (uint8_t)points_d;

    // Log every change in the touch count so the gesture can be traced live.
    if (points != s_last_logged_points) {
        ESP_LOGI(TAG, "touchPoints %u->%u (peak=%u)",
                 s_last_logged_points, points, s_max_points);
        s_last_logged_points = points;
    }

    // Gesture still in progress: track the highest finger count seen.
    if (points > 0) {
        if (points > s_max_points) {
            s_max_points = points;
        }
        return;
    }

    // points == 0: all fingers lifted -> resolve the gesture by its peak count.
    uint8_t peak = s_max_points;
    s_max_points = 0;

    if (peak < MULTI_FINGER_MIN_FINGERS || peak > MULTI_FINGER_MAX_FINGERS) {
        return;
    }
    uint8_t action = s_actions[peak - MULTI_FINGER_MIN_FINGERS];
    if (action == MULTI_FINGER_ACTION_NONE) {
        return;
    }

    int64_t now = esp_timer_get_time();
    if (now - s_last_fire_us < REFIRE_LOCKOUT_US) {
        ESP_LOGW(TAG, "%u-finger tap ignored: in %lld ms lockout", peak,
                 (long long)((REFIRE_LOCKOUT_US - (now - s_last_fire_us)) / 1000));
        return;
    }

    s_last_fire_us = now;
    ESP_LOGW(TAG, "%u-finger tap detected -> firing action %u", peak, action);
    fire_action(peak, action);
}

static void multi_finger_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode != VC_CMD_MULTI_FINGER_ACTION) {
        return;
    }
    // value packs the finger count in the high byte and the action in the low.
    set_action((uint8_t)(value >> 8), (uint8_t)(value & 0xFF));
}

static const char *const multi_finger_subs[] = {
    "UI_status2",
    NULL,
};

automation_t multi_finger_automation = {
    .name           = "multi_finger",
    .subscribe      = multi_finger_subs,
    .tick_period_ms = 0,
    .init           = multi_finger_init,
    .on_frame       = multi_finger_on_frame,
    .on_config      = multi_finger_on_config,
};
