// Battery-preheat automation. Fakes a UI_tripPlanning (0x082) frame so the car
// thinks it is navigating to a Supercharger and preconditions/heats the battery.
//
// Runs on the shared automation esp_timer (tick_period_ms = 100): NO dedicated
// FreeRTOS task. Each tick read-modify-writes the car's live UI_tripPlanning
// frame and forces the preheat fields via named DBC signals.

#include "automation.h"
#include "battery_preheat.h"
#include "vehicle_control.h"  // VC_CMD_BATTERY_PREHEAT
#include "dbc.h"

#include "esp_log.h"

static const char *TAG = "bat_preheat";

#define BUS  1

// Log roughly every 2 s (every 20 ticks at 100ms) so the bus log isn't flooded.
#define PREHEAT_LOG_EVERY  20

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
    // The DBC value cache keeps the live UI_tripPlanning frame automatically
    // (used as the read-modify-write base in on_tick); no watch needed.
    ESP_LOGI(TAG, "battery preheat injector ready (disabled)");
}

static void battery_preheat_on_tick(automation_t *self)
{
    (void)self;
    if (!s_enabled) {
        return;
    }

    // Read-modify-write the car's own live frame so the other fields/bytes stay
    // consistent with whatever it is broadcasting (and bit0 UI_tripPlanningActive
    // is left untouched). The car always emits UI_tripPlanning on the bus.
    can_frame_t f;
    if (can_frame_live(BUS, "UI_tripPlanning", &f) != ESP_OK) {
        return;
    }

    // Force the preheat fields. Values captured from a real car auto-preheating
    // while navigating to a Supercharger (byte0 == 0xAE, byte1 == 0x50,
    // byte2 == 0xC5). source: https://www.teslaownersonline.com/threads/diagnostic-port-and-data-access.7502/page-37
    dbc_msg_t m = dbc_msg("UI_tripPlanning");
    dbc_pack(f.data, dbc_sig(m, "UI_navToSupercharger"),           1);
    dbc_pack(f.data, dbc_sig(m, "UI_navSuperchargerType"),         3);
    dbc_pack(f.data, dbc_sig(m, "UI_batteryPreconditioningState"), 1);
    dbc_pack(f.data, dbc_sig(m, "UI_requestBatteryHeating"),       1);
    dbc_pack(f.data, dbc_sig(m, "UI_someValueA"),                  0x50);
    dbc_pack(f.data, dbc_sig(m, "UI_someValueB"),                  0xC5);

    esp_err_t err = can_frame_send(BUS, "UI_tripPlanning", &f);

    static uint32_t count = 0;
    if ((count++ % PREHEAT_LOG_EVERY) == 0) {
        ESP_LOGI(TAG, "TX 0x082 %02X %02X %02X %02X %02X %02X %02X %02X (%s)",
                 f.data[0], f.data[1], f.data[2],
                 f.data[3], f.data[4], f.data[5],
                 f.data[6], f.data[7],
                 err == ESP_OK ? "ok" : "SEND-FAIL");
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
    .tick_period_ms = 100,
    .init           = battery_preheat_init,
    .on_tick        = battery_preheat_on_tick,
    .on_config      = battery_preheat_on_config,
};
