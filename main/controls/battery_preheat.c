// Battery-preheat automation. Fakes a UI_tripPlanning (0x082) frame so the car
// thinks it is navigating to a Supercharger and preconditions/heats the battery.
//
// Runs on the shared automation esp_timer (tick_period_ms = 100): NO dedicated
// FreeRTOS task. Each tick read-modify-writes the car's live UI_tripPlanning
// frame and forces the preheat fields.
//
// The magic payload spans unnamed/reserved bits the published Tesla DBC gets
// wrong, so it is written as raw bytes via the can_frame_* helpers rather than
// through named DBC signals.

#include "automation.h"
#include "battery_preheat.h"
#include "vehicle_control.h"  // VC_CMD_BATTERY_PREHEAT
#include "dbc.h"

#include "esp_log.h"
#include <string.h>

static const char *TAG = "bat_preheat";

#define BUS  1

// Log roughly every 2 s (every 20 ticks at 100ms) so the bus log isn't flooded.
#define PREHEAT_LOG_EVERY  20

// Captured from a real car auto-preheating while navigating to a Supercharger.
// source: https://www.teslaownersonline.com/threads/diagnostic-port-and-data-access.7502/page-37
#define PREHEAT_D0_KEEP   0x01u  // preserve bit0 (tripPlanningActive) from the car
#define PREHEAT_D0_SET    0xAEu  // navSC=1, scType=3, precond=1, reqHeating=1
#define PREHEAT_VAL_A     0x50u
#define PREHEAT_VAL_B     0xC5u

static const uint8_t PREHEAT_DEFAULT[8] = {
    0xAE, 0x50, 0xC5, 0x80, 0xFF, 0x03, 0x00, 0x80,
};

static volatile bool s_enabled = false;

bool battery_preheat_enabled(void)
{
    return s_enabled;
}

// ---- Automation hooks ----
static void battery_preheat_init(automation_t *self)
{
    (void)self;
    s_enabled = false;
    // Keep a snapshot of the live UI_tripPlanning frame for read-modify-write
    // (subscribe is NULL, so register the watch explicitly).
    dbc_watch(BUS, "UI_tripPlanning");
    ESP_LOGI(TAG, "battery preheat injector ready (disabled)");
}

static void battery_preheat_on_tick(automation_t *self)
{
    (void)self;
    if (!s_enabled) {
        return;
    }

    // Prefer the car's own live frame so data[3..7] stays consistent with
    // whatever it is broadcasting; fall back to the captured default bytes.
    can_frame_t f;
    bool live = (can_frame_live(BUS, "UI_tripPlanning", &f) == ESP_OK);
    if (!live) {
        can_frame_init("UI_tripPlanning", &f);
        memcpy(f.data, PREHEAT_DEFAULT, sizeof(PREHEAT_DEFAULT));
    }

    f.data[0] = (f.data[0] & PREHEAT_D0_KEEP) |
                (PREHEAT_D0_SET & ~PREHEAT_D0_KEEP);
    f.data[1] = PREHEAT_VAL_A;
    f.data[2] = PREHEAT_VAL_B;

    esp_err_t err = can_frame_send(BUS, "UI_tripPlanning", &f);

    static uint32_t count = 0;
    if ((count++ % PREHEAT_LOG_EVERY) == 0) {
        ESP_LOGI(TAG, "TX 0x082 %02X %02X %02X %02X %02X %02X %02X %02X (%s, %s)",
                 f.data[0], f.data[1], f.data[2],
                 f.data[3], f.data[4], f.data[5],
                 f.data[6], f.data[7],
                 live ? "live" : "synth", err == ESP_OK ? "ok" : "SEND-FAIL");
    }
}

static void battery_preheat_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode != VC_CMD_BATTERY_PREHEAT) {
        return;
    }
    bool enable = (value != 0);
    if (enable != s_enabled) {
        ESP_LOGW(TAG, "UI_tripPlanning preheat injection %s",
                 enable ? "ENABLED" : "disabled");
    }
    s_enabled = enable;
}

automation_t battery_preheat_automation = {
    .name           = "battery_preheat",
    .subscribe      = NULL,
    .tick_period_ms = 100,
    .init           = battery_preheat_init,
    .on_tick        = battery_preheat_on_tick,
    .on_config      = battery_preheat_on_config,
};
