#include "battery_preheat.h"
#include "can_manager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "bat_preheat";

#define BUS                 1
#define UI_TRIPPLANNING_ID  0x082

// How often to push the fake UI_tripPlanning frame while preheat is enabled,
// and how slowly to poll the enable flag while idle. The car broadcasts this
// cyclically; we transmit at a steady 10 Hz so our value wins on the bus.
#define PREHEAT_TX_PERIOD_MS   100
#define PREHEAT_IDLE_POLL_MS   200
// Log roughly every 2 s (every 20 transmits) so the bus log isn't flooded.
#define PREHEAT_LOG_EVERY      20

// Captured from a real car auto-preheating while navigating to a Supercharger.
// source: https://www.teslaownersonline.com/threads/diagnostic-port-and-data-access.7502/page-37?utm_source=google&utm_medium=organic
#define PREHEAT_D0_KEEP   0x01u  // preserve bit0 (tripPlanningActive) from the car
#define PREHEAT_D0_SET    0xAEu  // navSC=1, scType=3, precond=1, reqHeating=1
#define PREHEAT_VAL_A     0x50u
#define PREHEAT_VAL_B     0xC5u

static const uint8_t PREHEAT_DEFAULT[8] = {
    0xAE, 0x50, 0xC5, 0x80, 0xFF, 0x03, 0x00, 0x80,
};

static volatile bool s_enabled = false;

void battery_preheat_set(bool enable)
{
    if (enable != s_enabled) {
        ESP_LOGW(TAG, "UI_tripPlanning preheat injection %s",
                 enable ? "ENABLED" : "disabled");
    }
    s_enabled = enable;
}

bool battery_preheat_enabled(void)
{
    return s_enabled;
}

static void preheat_task(void *arg)
{
    (void)arg;
    uint32_t count = 0;
    while (true) {
        if (!s_enabled) {
            count = 0;  // log immediately on the next enable
            vTaskDelay(pdMS_TO_TICKS(PREHEAT_IDLE_POLL_MS));
            continue;
        }

        // Prefer the car's own live frame so data[3..7] stays consistent with
        // whatever it is currently broadcasting; fall back to captured bytes.
        can_frame_t out;
        bool live = (can_manager_get_frame(BUS, UI_TRIPPLANNING_ID, &out) == ESP_OK);
        if (!live) {
            out.id = UI_TRIPPLANNING_ID;
            out.dlc = 8;
            memcpy(out.data, PREHEAT_DEFAULT, sizeof(PREHEAT_DEFAULT));
        }
        out.data[0] = (out.data[0] & PREHEAT_D0_KEEP) | (PREHEAT_D0_SET & ~PREHEAT_D0_KEEP);
        out.data[1] = PREHEAT_VAL_A;
        out.data[2] = PREHEAT_VAL_B;

        esp_err_t err = can_manager_send(BUS, &out);

        if ((count++ % PREHEAT_LOG_EVERY) == 0) {
            ESP_LOGI(TAG,
                     "TX 0x%03X %02X %02X %02X %02X %02X %02X %02X %02X (%s, %s)",
                     UI_TRIPPLANNING_ID, out.data[0], out.data[1], out.data[2],
                     out.data[3], out.data[4], out.data[5], out.data[6], out.data[7],
                     live ? "live" : "synth", err == ESP_OK ? "ok" : "SEND-FAIL");
        }

        vTaskDelay(pdMS_TO_TICKS(PREHEAT_TX_PERIOD_MS));
    }
}

esp_err_t battery_preheat_init(void)
{
    s_enabled = false;
    // Keep a snapshot of the live UI_tripPlanning frame for read-modify-write.
    can_manager_watch_frame(BUS, UI_TRIPPLANNING_ID);
    if (xTaskCreatePinnedToCore(preheat_task, "bat_preheat", 4096, NULL, 5, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "battery preheat injector ready (bus %d id 0x%03X, %dms cycle, disabled)",
             BUS, UI_TRIPPLANNING_ID, PREHEAT_TX_PERIOD_MS);
    return ESP_OK;
}
