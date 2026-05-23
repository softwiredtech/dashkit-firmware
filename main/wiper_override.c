#include "wiper_override.h"
#include "can_manager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>

static const char *TAG = "wiper_ovr";

#define WIPER_SPEED_OFF      0
#define WIPER_SPEED_INVALID  15

// Number of spoofed frames to fire right after each received DAS frame.
// One is usually enough (BCM treats latest received as authoritative); a
// small burst hedges against transient CAN errors and any "N consecutive
// frames required to change state" debouncer in the BCM. The whole burst
// finishes in a few ms, well before Tesla's next ~100 ms frame.
#define BURST_FRAMES         3
#define BURST_SPACING_MS     2

// Signal positions (intel byte order):
//   DAS_wiperSpeed:            byte 0, high nibble
//   DAS_bodyControlsCounter:   byte 6, high nibble
//   DAS_bodyControlsChecksum:  byte 7 (full)

static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_trigger;
static can_frame_t       s_template;
static uint8_t           s_base_counter;     // counter value from latest Tesla frame
static bool              s_have_template = false;
static uint8_t           s_last_wiper = 0xFF;
static bool              s_override_active = false;

// Tesla DAS_bodyControls checksum: (id_lo + id_hi + sum(data[0..6])) mod 256
static uint8_t das_bc_checksum(const uint8_t *data)
{
    uint16_t sum = (DAS_BODY_CONTROLS_ID & 0xFF) + ((DAS_BODY_CONTROLS_ID >> 8) & 0xFF);
    for (int i = 0; i < 7; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void wiper_override_on_rx(const can_tagged_frame_t *frame)
{
    if (frame->bus_id != DAS_BODY_CONTROLS_BUS) return;
    if (frame->frame.id != DAS_BODY_CONTROLS_ID) return;
    if (frame->frame.dlc < DAS_BODY_CONTROLS_DLC) return;

    uint8_t wiper = (frame->frame.data[0] >> 4) & 0x0F;
    bool fire = false;

    xSemaphoreTake(s_lock, portMAX_DELAY);

    // Only treat frames with wiperSpeed != 15 as genuine Tesla frames.
    // (Our own spoof has wiperSpeed=15; the MCP doesn't loop TX -> RX but
    // be defensive in case of future bus echo / multi-listener setups.)
    if (wiper != WIPER_SPEED_INVALID) {
        s_template = frame->frame;
        s_have_template = true;
        s_base_counter = (frame->frame.data[6] >> 4) & 0x0F;

        // 15 -> 0 transition latches the override on.
        if (s_last_wiper == WIPER_SPEED_INVALID && wiper == WIPER_SPEED_OFF
            && !s_override_active) {
            ESP_LOGI(TAG, "DAS_wiperSpeed 15 -> 0 detected, activating override");
            s_override_active = true;
        }

        // Every legit Tesla frame while the override is active triggers a
        // reactive spoof burst.
        if (s_override_active) {
            fire = true;
        }
    }
    s_last_wiper = wiper;

    xSemaphoreGive(s_lock);

    if (fire) {
        // Binary semaphore: extra givens are coalesced, which is fine -
        // each wakeup will read the latest template under the lock.
        xSemaphoreGive(s_trigger);
    }
}

static void wiper_override_task(void *arg)
{
    (void)arg;
    while (true) {
        if (xSemaphoreTake(s_trigger, portMAX_DELAY) != pdTRUE) continue;

        can_frame_t base;
        uint8_t base_counter;

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (!s_override_active || !s_have_template) {
            xSemaphoreGive(s_lock);
            continue;
        }
        base = s_template;
        base_counter = s_base_counter;
        xSemaphoreGive(s_lock);

        for (int i = 1; i <= BURST_FRAMES; i++) {
            can_frame_t f = base;
            f.dlc = DAS_BODY_CONTROLS_DLC;
            // Force DAS_wiperSpeed = 15 (high nibble of byte 0)
            f.data[0] = (f.data[0] & 0x0F) | (WIPER_SPEED_INVALID << 4);
            // Stamp counter = Tesla's counter + i (mod 16)
            uint8_t c = (uint8_t)((base_counter + i) & 0x0F);
            f.data[6] = (f.data[6] & 0x0F) | (c << 4);
            // Recompute checksum over the modified payload
            f.data[7] = das_bc_checksum(f.data);

            esp_err_t err = can_manager_send(DAS_BODY_CONTROLS_BUS, &f);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "send failed: %s", esp_err_to_name(err));
            }

            if (i < BURST_FRAMES) {
                vTaskDelay(pdMS_TO_TICKS(BURST_SPACING_MS));
            }
        }
    }
}

esp_err_t wiper_override_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    s_trigger = xSemaphoreCreateBinary();
    if (!s_trigger) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(
        wiper_override_task, "wiper_ovr", 4096, NULL, 4, NULL, 0);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Wiper override armed (reactive, waiting for 15 -> 0)");
    return ESP_OK;
}
