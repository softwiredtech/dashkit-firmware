// Sport kick-down: while the accelerator is past the threshold in Drive,
// RMW-inject UI_pedalMap=SPORT onto the car's live UI_powertrainControl
// (0x334) so every other signal stays as the car emits it. Stops once the
// pedal drops back below threshold-hysteresis, the car leaves Drive, or the
// driver changes the pedal map from the UI.

#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = "sport_kd";

#define PEDAL_BUS      0
#define PEDAL_MSG      "DI_systemStatus"
#define PEDAL_ID       0x118
#define PEDAL_SNA      255

#define GEAR_DRIVE     4

#define PT_BUS         1
#define PT_MSG         "UI_powertrainControl"
#define PT_ID          0x334   // was 0x313 before Tesla FW 2026.x
#define PT_SIG         "UI_pedalMap"

#define MAP_CHILL      0
#define MAP_SPORT      1

#define THRESHOLD_DEFAULT_PCT  80
#define THRESHOLD_MIN_PCT      10
#define THRESHOLD_MAX_PCT      95
#define RELEASE_HYST_PCT       10

#define INJECT_MS      10   // car's 0x334 is ~500ms; keep its CHILL frames from landing
#define TX_LOG_EVERY   200  // ~2s at 10ms

#define NVS_NAMESPACE     "sport_kd"
#define NVS_KEY_ENABLED   "en"
#define NVS_KEY_THRESHOLD "thr"

static volatile bool    s_enabled = false;
static volatile uint8_t s_threshold_pct = THRESHOLD_DEFAULT_PCT;

static volatile bool s_above = false;   // pedal latch with hysteresis
static volatile bool s_active = false;
static volatile int  s_baseline = -1;   // car's UI_pedalMap when injection started

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
        err = nvs_set_u8(nvs, NVS_KEY_THRESHOLD, s_threshold_pct);
    }
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
    if (nvs_get_u8(nvs, NVS_KEY_THRESHOLD, &v) == ESP_OK
        && v >= THRESHOLD_MIN_PCT && v <= THRESHOLD_MAX_PCT) {
        s_threshold_pct = v;
    }
    nvs_close(nvs);
}

static void stop_injection(const char *why)
{
    if (!s_active) {
        return;
    }
    s_active = false;
    ESP_LOGW(TAG, "STOP injection (%s)", why);
}

static void start_injection(double pedal_pct)
{
    can_frame_t f;
    if (can_frame_live(PT_BUS, PT_MSG, &f) != ESP_OK) {
        ESP_LOGW(TAG, "pedal %.0f%% but no live 0x334 frame seen yet -- cannot inject",
                 pedal_pct);
        return;
    }
    double map;
    if (can_get(PT_BUS, PT_MSG, PT_SIG, &map, false) != ESP_OK) {
        return;
    }
    if ((int)map == MAP_SPORT) {
        ESP_LOGI(TAG, "pedal %.0f%% but car already in SPORT -- nothing to do", pedal_pct);
        return;
    }
    dbc_counter_seed_from_bus(PT_BUS, PT_MSG);
    s_baseline = (int)map;
    s_active = true;
    ESP_LOGW(TAG, "START injection: pedal %.0f%% >= %u%%, car map=%d -> SPORT @ %dms",
             pedal_pct, s_threshold_pct, s_baseline, INJECT_MS);
}

static void sport_kickdown_init(automation_t *self)
{
    (void)self;
    load_config();
    s_above = false;
    s_active = false;
    xTaskCreatePinnedToCore(inject_task, "sport_kd_inj", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "sport kick-down READY (%s), threshold=%u%% release=%u%% inject=%dms",
             s_enabled ? "ENABLED" : "disabled",
             s_threshold_pct, s_threshold_pct - RELEASE_HYST_PCT, INJECT_MS);
}

static void on_pedal_frame(void)
{
    double raw, gear;
    if (can_get(PEDAL_BUS, PEDAL_MSG, "DI_accelPedalPos", &raw, false) != ESP_OK
        || can_get(PEDAL_BUS, PEDAL_MSG, "DI_gear", &gear, false) != ESP_OK) {
        return;
    }
    if ((int)raw == PEDAL_SNA) {
        return;
    }
    double pct = raw * 0.4;
    bool in_drive = ((int)gear == GEAR_DRIVE);

    if (s_active && !in_drive) {
        stop_injection("left Drive");
    }

    if (!s_above) {
        if (pct >= s_threshold_pct) {
            s_above = true;
            if (!s_enabled) {
                ESP_LOGI(TAG, "pedal %.0f%% >= %u%%, but disabled -- ignoring", pct, s_threshold_pct);
            } else if (!in_drive) {
                ESP_LOGI(TAG, "pedal %.0f%% >= %u%%, but gear=%d (not D) -- ignoring",
                         pct, s_threshold_pct, (int)gear);
            } else {
                start_injection(pct);
            }
        }
    } else if (pct < s_threshold_pct - RELEASE_HYST_PCT) {
        s_above = false;
        if (s_active) {
            ESP_LOGI(TAG, "pedal %.0f%% < %u%%", pct, s_threshold_pct - RELEASE_HYST_PCT);
            stop_injection("pedal released");
        }
    }
}

// Log the car's 0x334 whenever its payload changes on a bus, ignoring the
// rolling counter (byte 6 high nibble) and checksum (byte 7). Decodes the frame
// in hand: the value cache only holds the copy from the message's own bus.
static void log_pt_change(uint8_t bus, const can_frame_t *f)
{
    static uint8_t last[2][7];
    static bool    have_last[2];
    if (bus > 1) return;
    uint8_t cur[7];
    memcpy(cur, f->data, 7);
    cur[6] &= 0x0F;
    if (have_last[bus] && memcmp(cur, last[bus], sizeof(cur)) == 0) {
        return;
    }
    memcpy(last[bus], cur, sizeof(cur));
    have_last[bus] = true;
    static const char *names[] = { "CHILL", "SPORT", "PERFORMANCE", "?" };
    int map = (int)dbc_unpack(f->data, dbc_sig(dbc_msg(PT_MSG), PT_SIG));
    ESP_LOGI(TAG, "RX bus%u 0x334 %02X %02X %02X %02X %02X %02X %02X %02X  UI_pedalMap=%d (%s)",
             bus, f->data[0], f->data[1], f->data[2], f->data[3],
             f->data[4], f->data[5], f->data[6], f->data[7],
             map, names[map & 3]);
}

static void sport_kickdown_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;
    if (frame->bus_id == PEDAL_BUS && frame->frame.id == PEDAL_ID) {
        on_pedal_frame();
        return;
    }
    if (frame->frame.id == PT_ID) {
        log_pt_change(frame->bus_id, &frame->frame);
        double map;
        if (s_active && can_get(PT_BUS, PT_MSG, PT_SIG, &map, false) == ESP_OK
            && (int)map != s_baseline) {
            ESP_LOGI(TAG, "car map %d -> %d", s_baseline, (int)map);
            stop_injection("driver changed pedal map");
        }
    }
}

static void inject_task(void *arg)
{
    (void)arg;
    dbc_msg_t m = dbc_msg(PT_MSG);
    dbc_sig_t sig_map = dbc_sig(m, PT_SIG);
    uint32_t count = 0;

    while (true) {
        if (s_active) {
            can_frame_t f;
            if (can_frame_live(PT_BUS, PT_MSG, &f) != ESP_OK) {
                stop_injection("live frame lost");
            } else {
                dbc_pack(f.data, sig_map, MAP_SPORT);
                esp_err_t err = can_frame_send(PT_BUS, PT_MSG, &f);
                if ((count++ % TX_LOG_EVERY) == 0) {
                    ESP_LOGI(TAG, "TX 0x334 %02X %02X %02X %02X %02X %02X %02X %02X (%s)",
                             f.data[0], f.data[1], f.data[2], f.data[3],
                             f.data[4], f.data[5], f.data[6], f.data[7],
                             err == ESP_OK ? "ok" : "SEND-FAIL");
                }
            }
        } else {
            count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(INJECT_MS));
    }
}

static void sport_kickdown_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode == VC_CMD_SPORT_KICKDOWN_ENABLE) {
        bool enabled = (value != 0);
        if (enabled != s_enabled) {
            ESP_LOGW(TAG, "automation %s", enabled ? "ENABLED" : "disabled");
            s_enabled = enabled;
            save_config();
            if (!enabled) {
                stop_injection("automation disabled");
            }
        }
    } else if (opcode == VC_CMD_SPORT_KICKDOWN_THRESHOLD) {
        uint8_t pct = (value < THRESHOLD_MIN_PCT) ? THRESHOLD_MIN_PCT
                    : (value > THRESHOLD_MAX_PCT) ? THRESHOLD_MAX_PCT
                    : (uint8_t)value;
        if (pct != s_threshold_pct) {
            ESP_LOGW(TAG, "threshold %u%% -> %u%% (release %u%%)",
                     s_threshold_pct, pct, pct - RELEASE_HYST_PCT);
            s_threshold_pct = pct;
            save_config();
        }
    }
}

automation_t sport_kickdown_automation = {
    .name           = "sport_kickdown",
    .tick_period_ms = 0,
    .init           = sport_kickdown_init,
    .on_frame       = sport_kickdown_on_frame,
    .on_config      = sport_kickdown_on_config,
};
