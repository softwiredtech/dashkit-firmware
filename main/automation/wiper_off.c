#include "wiper_off.h"
#include "can_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "wiper_off";

// Persist the enabled flag so it survives a reboot/power-cycle even before the
// Android app reconnects and re-syncs it.
#define NVS_NAMESPACE   "wiper_off"
#define NVS_KEY_ENABLED "en"

// ---- CAN IDs and bus ----
#define BUS                       1
#define SCCM_LEFT_STALK_ID       0x249
#define SCCM_LEFT_STALK_DLC      3
#define VCLEFT_SWITCH_STATUS_ID   0x3C2
#define VCLEFT_SWITCH_STATUS_DLC  8
#define DAS_BODY_CONTROLS_ID      0x3E9  // DAS_bodyControls (1001 decimal)

// How far ahead of the last observed SCCM counter to start injecting.
#define COUNTER_OFFSET  1

// Tesla checksum: sum of address bytes + all data bytes except the checksum byte.
// Matches opendbc / panda steering_button_mode.py implementation.
static uint8_t tesla_checksum(uint32_t addr, const uint8_t *dat, uint8_t len, uint8_t cksum_byte)
{
    uint8_t sum = (addr & 0xFF) + ((addr >> 8) & 0xFF);
    for (uint8_t i = 0; i < len; i++) {
        if (i != cksum_byte) sum += dat[i];
    }
    return sum;
}

static void build_left_stalk(uint8_t out[3], uint8_t counter,
                              uint8_t wash_wipe)
{
    out[1] = (counter & 0x0F) | ((wash_wipe & 0x03) << 6);
    out[2] = 0;
    out[0] = tesla_checksum(SCCM_LEFT_STALK_ID, out, 3, 0);
}

// ---- State ----
static volatile uint8_t s_last_counter = 0;
static volatile uint8_t s_last_wiper_speed = 0xFF;  // 0xFF = unknown
// Whether the automation is armed. Off by default (matches the Android app's
// default); the app enables it over BLE and re-syncs on each connect.
static volatile bool    s_enabled = false;

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

void wiper_off_set_enabled(bool enabled)
{
    if (enabled != s_enabled) {
        ESP_LOGW(TAG, "automation %s", enabled ? "enabled" : "disabled");
        s_enabled = enabled;
        save_enabled(enabled);
    }
}

// ---- Wiper-off sequence (runs in its own task) ----
static void wiper_off_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "starting");

    uint8_t counter = (s_last_counter + COUNTER_OFFSET) & 0x0F;

    // Phase 1: hold wiper button (1ST_DETENT) for 300ms at 100Hz
    int64_t end = esp_timer_get_time() + 300000;
    while (esp_timer_get_time() < end) {
        can_frame_t f = { .id = SCCM_LEFT_STALK_ID, .dlc = SCCM_LEFT_STALK_DLC };
        build_left_stalk(f.data, counter, 1);
        can_manager_send(BUS, &f);
        counter = (counter + 1) & 0x0F;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Gap: let the car register the button hold
    vTaskDelay(pdMS_TO_TICKS(100));

    // Phase 2: scroll-down ticks (send twice for reliability)
    for (int i = 0; i < 2; i++) {
        can_frame_t scroll = { .id = VCLEFT_SWITCH_STATUS_ID, .dlc = VCLEFT_SWITCH_STATUS_DLC };
        memset(scroll.data, 0, 8);
        scroll.data[0] = 0x01;   // mux index 1
        scroll.data[2] = 0x3F;   // swcLeftScrollTicks = -1 (6-bit signed)
        can_manager_send(BUS, &scroll);
        if (i == 0) vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "scroll-down sent");

    ESP_LOGI(TAG, "done");
    vTaskDelete(NULL);
}

// ---- RX observation (called from CAN driver RX task) ----
void wiper_off_observe(const can_tagged_frame_t *frame)
{
    if (!frame || frame->bus_id != BUS) return;

    // Track SCCM counter
    if (frame->frame.id == SCCM_LEFT_STALK_ID && frame->frame.dlc >= 2) {
        s_last_counter = frame->frame.data[1] & 0x0F;
    }

    // Trigger: DAS_wiperSpeed transitions from 15 to 0. Keep tracking the speed
    // even when disabled so we don't fire on a stale edge right after enabling.
    if (frame->frame.id == DAS_BODY_CONTROLS_ID && frame->frame.dlc >= 1) {
        uint8_t speed = (frame->frame.data[0] >> 4) & 0x0F;
        if (s_enabled && s_last_wiper_speed == 15 && speed == 0) {
            ESP_LOGW(TAG, "DAS_wiperSpeed 15->0, triggering");
            xTaskCreatePinnedToCore(wiper_off_task, "wiper_off",
                                    4096, NULL, 5, NULL, 0);
        }
        s_last_wiper_speed = speed;
    }
}

esp_err_t wiper_off_init(void)
{
    load_enabled();
    ESP_LOGI(TAG, "auto wiper-off initialized (%s)",
             s_enabled ? "enabled" : "disabled");
    return ESP_OK;
}
