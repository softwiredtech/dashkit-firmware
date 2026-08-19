// Keeps climate on for KEEP_DURATION after the driver leaves, so the cabin is
// still conditioned on return. Trigger: UI_userPresent 1->0. Control: replay the
// last climate-on UI_hvacRequest at 100Hz (dedicated task, mirroring the proven
// rear-fan injector) with power ON + KeepClimateOn.

#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"  // VC_CMD_CLIMATE_KEEP_ENABLE

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "climate_keep";

#define BUS  1

#define PRESENCE_MSG   "UI_vehicleControl2"
#define PRESENCE_SIG   "UI_userPresent"
#define PRESENCE_ID    0x3B3

#define HVAC_MSG       "UI_hvacRequest"
#define HVAC_ID        0x2F3

#define KEEP_DEFAULT_MIN   5
#define KEEP_MIN_MINUTES   1
#define KEEP_MAX_MINUTES   60

#define HVAC_POWER_ON      1  // UI_hvacReqUserPowerState: 1 = ON, 0 = OFF
#define HVAC_POWER_OFF     0
#define HVAC_KEEP_OFF      0
// UI_hvacReqKeepClimateOn: no VAL_ table; guessed 1=Keep/2=Dog/3=Camp, unverified.
#define HVAC_KEEP_MODE     1

#define INJECT_MS      10   // 100Hz, matches the rear-fan injector on this frame
#define TX_LOG_EVERY   200  // ~2s at 10ms
#define SHUTOFF_REPS   20   // ~200ms burst commanding climate OFF at the deadline

// Reject a snapshot older than this: only keep climate that was running as you left.
#define SNAPSHOT_FRESH_US  (30LL * 1000 * 1000)  // 30 s

#define NVS_NAMESPACE   "climate_keep"
#define NVS_KEY_ENABLED "en"
#define NVS_KEY_MINUTES "min"

static volatile bool s_enabled = false;

static volatile uint16_t s_keep_minutes = KEEP_DEFAULT_MIN;

static volatile int s_present = -1;  // -1 = unknown

static can_frame_t       s_snapshot;
static volatile bool     s_snapshot_valid = false;
static volatile int64_t  s_snapshot_us = 0;

static volatile bool     s_active = false;
static volatile int64_t  s_deadline_us = 0;

static void climate_keep_inject_task(void *arg);

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
        err = nvs_set_u16(nvs, NVS_KEY_MINUTES, s_keep_minutes);
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
    uint8_t enabled = 0;
    if (nvs_get_u8(nvs, NVS_KEY_ENABLED, &enabled) == ESP_OK) {
        s_enabled = (enabled != 0);
    }
    uint16_t minutes = 0;
    if (nvs_get_u16(nvs, NVS_KEY_MINUTES, &minutes) == ESP_OK
        && minutes >= KEEP_MIN_MINUTES && minutes <= KEEP_MAX_MINUTES) {
        s_keep_minutes = minutes;
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

static void start_injection(void)
{
    if (!s_snapshot_valid) {
        ESP_LOGE(TAG, "no climate-on snapshot -- cannot keep, aborting");
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t age_us = now - s_snapshot_us;
    if (age_us > SNAPSHOT_FRESH_US) {
        ESP_LOGW(TAG, "last climate-on was %llds ago (>%llds) -- treating as OFF",
                 age_us / 1000000, SNAPSHOT_FRESH_US / 1000000);
        return;
    }
    ESP_LOGI(TAG, "snapshot fresh (%llds old)", age_us / 1000000);

    s_deadline_us = now + (int64_t)s_keep_minutes * 60 * 1000 * 1000;
    s_active = true;
    ESP_LOGW(TAG, "START keeping climate ON for %umin (L=0x%02X R=0x%02X power=0x%02X)",
             s_keep_minutes,
             s_snapshot.data[0], s_snapshot.data[1], s_snapshot.data[3]);
}

static void climate_keep_init(automation_t *self)
{
    (void)self;
    load_config();
    s_present = -1;
    s_snapshot_valid = false;
    s_active = false;
    xTaskCreatePinnedToCore(climate_keep_inject_task, "climate_keep_inj", 4096,
                            NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "climate-keep ready (%s), window=%umin, keep-mode=%d, inject=%dms",
             s_enabled ? "ENABLED" : "disabled",
             s_keep_minutes, HVAC_KEEP_MODE, INJECT_MS);
}

static void climate_keep_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;

    // Snapshot the live frame while climate is ON. Post-exit OFF frames have
    // power != ON so they never clobber it -- we keep the last running state.
    if (frame->frame.id == HVAC_ID) {
        double power;
        if (can_get(BUS, HVAC_MSG, "UI_hvacReqUserPowerState", &power, false) == ESP_OK
            && (int)power == HVAC_POWER_ON) {
            can_frame_t f;
            if (can_frame_live(BUS, HVAC_MSG, &f) == ESP_OK) {
                s_snapshot = f;
                s_snapshot_us = esp_timer_get_time();
                if (!s_snapshot_valid) {
                    ESP_LOGI(TAG, "captured climate-on snapshot (L=0x%02X R=0x%02X)",
                             f.data[0], f.data[1]);
                }
                s_snapshot_valid = true;
            }
        }
    }

    if (frame->frame.id != PRESENCE_ID) {
        return;
    }
    double p;
    if (can_get(BUS, PRESENCE_MSG, PRESENCE_SIG, &p, false) != ESP_OK) {
        return;
    }
    int present = (int)p ? 1 : 0;
    if (present == s_present) {
        return;
    }

    int prev = s_present;
    s_present = present;
    ESP_LOGI(TAG, "UI_userPresent %s -> %d",
             prev < 0 ? "(init)" : (prev ? "1" : "0"), present);

    if (prev == 1 && present == 0) {
        if (!s_enabled) {
            ESP_LOGI(TAG, "user left, but disabled -- ignoring");
            return;
        }
        if (!s_active) {
            start_injection();
        }
    } else if (present == 1 && s_active) {
        stop_injection("user returned");
    }
}

static void climate_keep_inject_task(void *arg)
{
    (void)arg;
    dbc_msg_t m = dbc_msg(HVAC_MSG);
    dbc_sig_t sig_power = dbc_sig(m, "UI_hvacReqUserPowerState");
    dbc_sig_t sig_keep  = dbc_sig(m, "UI_hvacReqKeepClimateOn");
    uint32_t count = 0;

    while (true) {
        if (s_active) {
            int64_t now = esp_timer_get_time();
            if (now >= s_deadline_us) {
                // Deadline: actively command climate OFF instead of going silent,
                // so a latched KeepClimateOn mode doesn't linger past the window.
                for (int i = 0; i < SHUTOFF_REPS; i++) {
                    can_frame_t f = s_snapshot;
                    dbc_pack(f.data, sig_power, HVAC_POWER_OFF);
                    dbc_pack(f.data, sig_keep,  HVAC_KEEP_OFF);
                    can_frame_send(BUS, HVAC_MSG, &f);
                    vTaskDelay(pdMS_TO_TICKS(INJECT_MS));
                }
                ESP_LOGW(TAG, "shutoff burst sent (%d frames)", SHUTOFF_REPS);
                stop_injection("keep window elapsed");
            } else if (!s_snapshot_valid) {
                stop_injection("snapshot lost");
            } else {
                can_frame_t f = s_snapshot;
                dbc_pack(f.data, sig_power, HVAC_POWER_ON);
                dbc_pack(f.data, sig_keep,  HVAC_KEEP_MODE);
                esp_err_t err = can_frame_send(BUS, HVAC_MSG, &f);

                if ((count++ % TX_LOG_EVERY) == 0) {
                    int remaining_s = (int)((s_deadline_us - now) / 1000000);
                    ESP_LOGI(TAG, "TX 0x2F3 %02X %02X %02X %02X %02X  %ds left (%s)",
                             f.data[0], f.data[1], f.data[2], f.data[3], f.data[4],
                             remaining_s, err == ESP_OK ? "ok" : "SEND-FAIL");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(INJECT_MS));
    }
}

static void climate_keep_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode == VC_CMD_CLIMATE_KEEP_ENABLE) {
        bool enabled = (value != 0);
        if (enabled != s_enabled) {
            ESP_LOGW(TAG, "automation %s", enabled ? "ENABLED" : "disabled");
            s_enabled = enabled;
            save_config();
            if (!enabled) {
                stop_injection("automation disabled");
            }
        }
    } else if (opcode == VC_CMD_CLIMATE_KEEP_DURATION) {
        uint16_t minutes = value;
        if (minutes < KEEP_MIN_MINUTES) {
            minutes = KEEP_MIN_MINUTES;
        } else if (minutes > KEEP_MAX_MINUTES) {
            minutes = KEEP_MAX_MINUTES;
        }
        if (minutes != s_keep_minutes) {
            ESP_LOGW(TAG, "keep window set to %umin", minutes);
            s_keep_minutes = minutes;
            save_config();
        }
    }
}

automation_t climate_keep_automation = {
    .name           = "climate_keep",
    .tick_period_ms = 0,
    .init           = climate_keep_init,
    .on_frame       = climate_keep_on_frame,
    .on_config      = climate_keep_on_config,
};
