// Turn signal off in Park: shifting into P with one turn signal blinking cancels it, the way Autopilot cancels
// its own (DAS_bodyControls.DAS_turnIndicatorRequest = CANCEL, one frame). Both sides blinking is the hazards:
// left alone. Off by default; enabled over BLE, persisted in NVS.

#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

static const char *TAG = "signal_off";

#define GEAR_BUS     0
#define LAMP_BUS     1
#define GEAR_P       1
#define LIT_HOLD_US  (1000 * 1000)   // a blinking lamp is dark ~0.35 s at a time
#define TRIES        3               // cancels, 1 s apart, while the lamp keeps blinking
#define NVS_NS       "signal_off"

static bool    s_enabled;
static int     s_gear = -1;
static int64_t s_lit_us[2];          // last time each side's lamp was seen lit (0 left, 1 right)
static int     s_tries;              // cancels still to send for this shift into P
static int64_t s_next_us;

static void lamp(int side, const char *msg, const char *sig, int64_t now)
{
    double v;
    if (can_get(LAMP_BUS, msg, sig, &v, false) == ESP_OK && v == 1.0) s_lit_us[side] = now;
}

static void signal_off_init(automation_t *self)
{
    (void)self;
    nvs_handle_t nvs;
    uint8_t en = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, "en", &en);
        nvs_close(nvs);
    }
    s_enabled = en;
    ESP_LOGI(TAG, "ready (%s)", s_enabled ? "enabled" : "disabled");
}

static void signal_off_tick(automation_t *self)
{
    (void)self;
    int64_t now = esp_timer_get_time();
    lamp(0, "VCLEFT_lightStatus", "VCLEFT_turnSignalStatus", now);
    lamp(1, "VCRIGHT_lightStatus", "VCRIGHT_turnSignalStatus", now);
    bool left = now - s_lit_us[0] < LIT_HOLD_US, right = now - s_lit_us[1] < LIT_HOLD_US;

    double g;
    int gear = can_msg_fresh("DI_systemStatus", 500)
               && can_get(GEAR_BUS, "DI_systemStatus", "DI_gear", &g, false) == ESP_OK ? (int)g : -1;
    if (gear == GEAR_P && s_gear >= 2 && s_gear <= 4) {   // R, N or D -> P
        s_tries = s_enabled ? TRIES : 0;
        s_next_us = now;
    }
    if (gear >= 0) s_gear = gear;
    if (gear != GEAR_P || left == right) s_tries = 0;     // moved off P, signal stopped, or hazards
    if (s_tries == 0 || now < s_next_us) return;

    // Continue the car's own counter so the frame isn't dropped as a repeat; the live frame keeps every other
    // request (headlights, wipers, hazards) as Autopilot last sent it.
    dbc_counter_seed_from_bus(LAMP_BUS, "DAS_bodyControls");
    esp_err_t err = can_send_live(LAMP_BUS, "DAS_bodyControls", "DAS_turnIndicatorRequest", 3, false);
    ESP_LOGI(TAG, "in Park with the %s signal on: cancel %d/%d (%s)", left ? "left" : "right",
             TRIES - s_tries + 1, TRIES, esp_err_to_name(err));
    s_tries--;
    s_next_us = now + 1000 * 1000;
}

static void signal_off_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode != VC_CMD_SIGNAL_OFF_ENABLE || s_enabled == (value != 0)) return;
    s_enabled = value != 0;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_u8(nvs, "en", s_enabled) == ESP_OK) nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGW(TAG, "automation %s", s_enabled ? "enabled" : "disabled");
}

automation_t signal_off_automation = {
    .name           = "signal_off",
    .tick_period_ms = 100,
    .init           = signal_off_init,
    .on_tick        = signal_off_tick,
    .on_config      = signal_off_on_config,
};
