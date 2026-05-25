#include "wiper_off.h"
#include "can_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "wiper_off";

// ---- CAN IDs and bus ----
#define BUS                       1
#define SCCM_LEFT_STALK_ID       0x249
#define SCCM_LEFT_STALK_DLC      3
#define VCLEFT_SWITCH_STATUS_ID   0x3C2
#define VCLEFT_SWITCH_STATUS_DLC  8
#define DAS_BODY_CONTROLS_ID      0x3E9  // DAS_bodyControls (1001 decimal)

// How far ahead of the last observed SCCM counter to start injecting.
#define COUNTER_OFFSET  1

// ---- Tesla checksum ----
// Seed table derived from 16 idle (wipe=0) SCCM frames.
static const uint8_t SCCM_SEED[16] = {
    0x9B, 0xE8, 0x2A, 0xD3, 0xD3, 0x83, 0x4C, 0x5E,
    0x3F, 0x5E, 0xE2, 0x28, 0x3A, 0x13, 0xAF, 0xCE
};
// Delta when wash_wipe = 1 (1ST_DETENT).
#define WIPE1_DELTA  0xF7

static void build_left_stalk(uint8_t out[3], uint8_t counter,
                              uint8_t wash_wipe)
{
    out[1] = (counter & 0x0F) | ((wash_wipe & 0x03) << 6);
    out[2] = 0;
    out[0] = SCCM_SEED[counter & 0x0F];
    if (wash_wipe == 1) out[0] += WIPE1_DELTA;
}

// ---- State ----
static volatile uint8_t s_last_counter = 0;
static volatile uint8_t s_last_wiper_speed = 0xFF;  // 0xFF = unknown

// ---- Wiper-off sequence (runs in its own task) ----
static void wiper_off_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "starting");

    uint8_t counter = (s_last_counter + COUNTER_OFFSET) & 0x0F;

    // Phase 1: hold wiper button (1ST_DETENT) for 200ms at 100Hz
    int64_t end = esp_timer_get_time() + 200000;
    while (esp_timer_get_time() < end) {
        can_frame_t f = { .id = SCCM_LEFT_STALK_ID, .dlc = SCCM_LEFT_STALK_DLC };
        build_left_stalk(f.data, counter, 1);
        can_manager_send(BUS, &f);
        counter = (counter + 1) & 0x0F;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Phase 2: scroll-down tick
    can_frame_t scroll = { .id = VCLEFT_SWITCH_STATUS_ID, .dlc = VCLEFT_SWITCH_STATUS_DLC };
    memset(scroll.data, 0, 8);
    scroll.data[0] = 0x01;   // mux index 1
    scroll.data[2] = 0x3F;   // swcLeftScrollTicks = -1 (6-bit signed)
    can_manager_send(BUS, &scroll);
    ESP_LOGI(TAG, "scroll-down sent");

    // Phase 3: brief hold with wiper button still pressed (100ms)
    end = esp_timer_get_time() + 100000;
    while (esp_timer_get_time() < end) {
        can_frame_t f = { .id = SCCM_LEFT_STALK_ID, .dlc = SCCM_LEFT_STALK_DLC };
        build_left_stalk(f.data, counter, 1);
        can_manager_send(BUS, &f);
        counter = (counter + 1) & 0x0F;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Phase 4: release
    can_frame_t rel = { .id = SCCM_LEFT_STALK_ID, .dlc = SCCM_LEFT_STALK_DLC };
    build_left_stalk(rel.data, counter, 0);
    can_manager_send(BUS, &rel);

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

    // Trigger: DAS_wiperSpeed transitions from 15 to 0
    if (frame->frame.id == DAS_BODY_CONTROLS_ID && frame->frame.dlc >= 1) {
        uint8_t speed = (frame->frame.data[0] >> 4) & 0x0F;
        if (s_last_wiper_speed == 15 && speed == 0) {
            ESP_LOGW(TAG, "DAS_wiperSpeed 15->0, triggering");
            xTaskCreatePinnedToCore(wiper_off_task, "wiper_off",
                                    4096, NULL, 5, NULL, 0);
        }
        s_last_wiper_speed = speed;
    }
}

esp_err_t wiper_off_init(void)
{
    ESP_LOGI(TAG, "auto wiper-off initialized");
    return ESP_OK;
}
