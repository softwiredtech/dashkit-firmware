// Battery-preheat automation. Fakes a UI_tripPlanning (0x347) frame so the car
// thinks it is navigating to a Supercharger and preconditions/heats the battery.
//
// CAN ID history: UI_tripPlanning lived at 0x082 until the 2026 Tesla firmware
// update moved it to 0x347 (839). The byte layout is unchanged. The car sends
// an idle frame `00 7F FF 80 FF 03 00 80` at 1 Hz; while preheating, a frame
// `AE 50 C8 28 FF 03 00 80` appears right after it, and the BMS/VCFRONT waste-heat
// request (0x241/0x268) starts ~1 s later (comma rlog, 2026-09-06).
//
// Runs on the shared automation esp_timer (tick_period_ms = 100): NO dedicated
// FreeRTOS task. Each tick read-modify-writes the car's live UI_tripPlanning
// frame and forces the preheat fields via named DBC signals.

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

// Captured 0x347 payload from a real car (post-2026 firmware) while its battery
// was preheating. The pre-update 0x082 capture was AE 50 C5 80 FF 03 00 80; only
// UI_someValueB (0xC5 -> 0xC8) and byte 3 (0x80 -> 0x28) differ.
static const uint8_t PREHEAT_DEFAULT[8] = {
    0xAE, 0x50, 0xC8, 0x28, 0xFF, 0x03, 0x00, 0x80,
};

// Values forced into the live frame every tick (see PREHEAT_DEFAULT).
#define PREHEAT_SC_TYPE       3      // UI_navSuperchargerType
#define PREHEAT_PRECOND_STATE 1      // UI_batteryPreconditioningState
#define PREHEAT_SOME_VALUE_A  0x50   // byte 1
#define PREHEAT_SOME_VALUE_B  0xC8   // byte 2
#define PREHEAT_SOME_VALUE_C  0x28   // byte 3

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

    can_frame_t f;
    bool live = (can_frame_live(BUS, "UI_tripPlanning", &f) == ESP_OK);
    if (!live) {
        if (can_frame_init("UI_tripPlanning", &f) != ESP_OK) {
            return;
        }
        memcpy(f.data, PREHEAT_DEFAULT, sizeof(PREHEAT_DEFAULT));
    }

    // Force the preheat fields so the frame matches the real car's active
    // 0x347 frame byte-for-byte in bytes 0..3 (AE 50 C8 28); bytes 4..7 are
    // kept from the live frame (FF 03 00 80 in both idle and active captures).
    // Original 0x082 reference: https://www.teslaownersonline.com/threads/diagnostic-port-and-data-access.7502/page-37
    dbc_msg_t m = dbc_msg("UI_tripPlanning");
    dbc_pack(f.data, dbc_sig(m, "UI_tripPlanningActive"),          0);
    dbc_pack(f.data, dbc_sig(m, "UI_navToSupercharger"),           1);
    dbc_pack(f.data, dbc_sig(m, "UI_navSuperchargerType"),         PREHEAT_SC_TYPE);
    dbc_pack(f.data, dbc_sig(m, "UI_batteryPreconditioningState"), PREHEAT_PRECOND_STATE);
    dbc_pack(f.data, dbc_sig(m, "UI_requestBatteryHeating"),       1);
    dbc_pack(f.data, dbc_sig(m, "UI_someValueA"),                  PREHEAT_SOME_VALUE_A);
    dbc_pack(f.data, dbc_sig(m, "UI_someValueB"),                  PREHEAT_SOME_VALUE_B);
    dbc_pack(f.data, dbc_sig(m, "UI_someValueC"),                  PREHEAT_SOME_VALUE_C);

    esp_err_t err = can_frame_send(BUS, "UI_tripPlanning", &f);

    static uint32_t count = 0;
    if ((count++ % PREHEAT_LOG_EVERY) == 0) {
        ESP_LOGI(TAG, "TX 0x347 %02X %02X %02X %02X %02X %02X %02X %02X (%s, %s)",
                 f.data[0], f.data[1], f.data[2],
                 f.data[3], f.data[4], f.data[5],
                 f.data[6], f.data[7],
                 live ? "live" : "synth",
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
