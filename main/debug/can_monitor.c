#include "can_monitor.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "can_mon";

// Tesla VEH bus: SCCM_leftStalk (DBC: BO_ 585 SCCM_leftStalk: 3 VEH)
// SG_ SCCM_leftStalkCounter : 8|4@1+  -> bits 8..11 little-endian
// i.e. the lower nibble of data[1].
#define MONITOR_BUS_ID        1
#define MONITOR_CAN_ID        0x249  // 585 decimal
#define MONITOR_QUEUE_DEPTH   16
#define STATS_INTERVAL_US     5000000ULL  // print rolling stats every 5 s

static QueueHandle_t s_mon_queue = NULL;

static inline uint8_t extract_counter(const can_frame_t *f)
{
    // SCCM_leftStalkCounter: 4 bits at bit offset 8 (little-endian).
    // That's the low nibble of byte 1.
    return (f->dlc >= 2) ? (f->data[1] & 0x0F) : 0;
}

static void monitor_task(void *arg)
{
    (void)arg;
    can_tagged_frame_t frame;
    int last_counter = -1;  // -1 = no previous value
    uint32_t last_ts_us = 0;

    // Rolling stats since last summary print.
    uint32_t window_frames  = 0;
    uint32_t window_jumps   = 0;  // # of non-sequential transitions
    uint32_t window_missed  = 0;  // estimated # of dropped frames
    // Lifetime totals.
    uint64_t total_frames   = 0;
    uint64_t total_jumps    = 0;
    uint64_t total_missed   = 0;
    uint64_t next_stats_us  = (uint64_t)esp_timer_get_time() + STATS_INTERVAL_US;

    while (true) {
        // Wake periodically even with no traffic so stats keep printing.
        BaseType_t got = xQueueReceive(s_mon_queue, &frame, pdMS_TO_TICKS(1000));

        if (got == pdTRUE) {
            uint8_t counter = extract_counter(&frame.frame);
            window_frames++;
            total_frames++;

            if (last_counter < 0) {
                ESP_LOGI(TAG, "ts=%lu us  SCCM_leftStalkCounter=%u  (first)",
                         (unsigned long)frame.timestamp_us, counter);
            } else if ((int)counter != last_counter) {
                int expected = (last_counter + 1) & 0x0F;
                int32_t delta_us = (int32_t)(frame.timestamp_us - last_ts_us);
                if ((int)counter == expected) {
                    // Normal increment, no log spam.
                } else {
                    // Number of skipped counter values (1..15).
                    uint32_t missed = (uint32_t)((counter - last_counter - 1) & 0x0F);
                    window_jumps++;
                    window_missed += missed;
                    total_jumps++;
                    total_missed += missed;
                    ESP_LOGW(TAG,
                             "ts=%lu us  dt=%ld us  counter %d -> %u  (JUMP, missed=%lu)",
                             (unsigned long)frame.timestamp_us,
                             (long)delta_us,
                             last_counter, counter,
                             (unsigned long)missed);
                }
            }
            // Repeated identical counter (counter == last_counter) is also
            // abnormal but we don't treat it as a drop; it would mean the
            // ECU re-sent the same frame or we double-received.

            last_counter = counter;
            last_ts_us = frame.timestamp_us;
        }

        uint64_t now_us = (uint64_t)esp_timer_get_time();
        if (now_us >= next_stats_us) {
            // expected frames in this window = received + missed
            uint32_t window_expected = window_frames + window_missed;
            float window_loss = window_expected
                ? (100.0f * (float)window_missed / (float)window_expected)
                : 0.0f;
            uint64_t total_expected = total_frames + total_missed;
            float total_loss = total_expected
                ? (100.0f * (float)total_missed / (float)total_expected)
                : 0.0f;

            ESP_LOGI(TAG,
                     "stats: window rx=%lu jumps=%lu missed=%lu loss=%.3f%% | "
                     "total rx=%llu jumps=%llu missed=%llu loss=%.3f%%",
                     (unsigned long)window_frames,
                     (unsigned long)window_jumps,
                     (unsigned long)window_missed,
                     window_loss,
                     (unsigned long long)total_frames,
                     (unsigned long long)total_jumps,
                     (unsigned long long)total_missed,
                     total_loss);

            window_frames = 0;
            window_jumps  = 0;
            window_missed = 0;
            next_stats_us = now_us + STATS_INTERVAL_US;
        }
    }
}

esp_err_t can_monitor_init(void)
{
    if (s_mon_queue) return ESP_OK;
    s_mon_queue = xQueueCreate(MONITOR_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
    if (!s_mon_queue) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(
        monitor_task, "can_mon", 4096, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        vQueueDelete(s_mon_queue);
        s_mon_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Monitoring bus %d id 0x%03X (SCCM_leftStalkCounter)",
             MONITOR_BUS_ID, MONITOR_CAN_ID);
    return ESP_OK;
}

void can_monitor_observe(const can_tagged_frame_t *frame)
{
    if (!s_mon_queue || !frame) return;
    if (frame->bus_id != MONITOR_BUS_ID) return;
    if (frame->frame.id != MONITOR_CAN_ID) return;
    // Stamp here since we run before the can_manager's own timestamping.
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    // Non-blocking: drop if queue is full (task should keep up easily).
    xQueueSend(s_mon_queue, &stamped, 0);
}
