#include "board.h"
#include "led.h"
#include "can_interface.h"
#include "can_manager.h"
#include "can_filter.h"
#include "dbc.h"
#include "automation_manager.h"
#include "vehicle_control.h"
#include "mcp251xfd.h"
#include "ble_server.h"
#include "ble_stream.h"
#include "ble_ota.h"
#include "ble_appchan.h"
#include "tesla_ble_adapter.h"
#include "tesla_ble_client.h"
#include "tesla_pairing.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#if !defined(CONFIG_BT_NIMBLE_NVS_PERSIST) || (CONFIG_BT_NIMBLE_NVS_PERSIST != 1)
#error "CONFIG_BT_NIMBLE_NVS_PERSIST must be enabled (bonds would be RAM-only)"
#endif

#if !defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) || (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE != 1)
#error "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE must be enabled (a crashing OTA image would brick the device)"
#endif

// Tesla BLE central client relies on the NimBLE central role (connect,
// discovery, write/indicate against the vehicle). No observer: the app
// provisions the car's address over the app channel.
#if defined(CONFIG_DASHKIT_TESLA_BLE) && \
    (!defined(CONFIG_BT_NIMBLE_ROLE_CENTRAL) || (CONFIG_BT_NIMBLE_ROLE_CENTRAL != 1))
#error "CONFIG_DASHKIT_TESLA_BLE requires CONFIG_BT_NIMBLE_ROLE_CENTRAL (set CONFIG_BT_NIMBLE_ROLE_CENTRAL=y)"
#endif

static const char *TAG = "main";

// Commit the running image after it has survived a stable boot window (OTA
// rollback: PENDING_VERIFY -> VALID, so the next reboot does not roll back).
#define MARK_VALID_DELAY_MS  10000

static void mark_app_valid_cb(void *arg)
{
    (void)arg;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mark app valid failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "app confirmed valid (OTA rollback cleared)");
    }
}

#define BLE_BATCH_MAX_FRAMES  16
#define BLE_BATCH_TIMEOUT_MS  20

// Which frames a device gets: the ids in its own CAN filter.
static bool want_frame(const can_tagged_frame_t *f, void *ctx)
{
    const ble_receiver_t *r = ctx;
    return can_filter_wants(r->slot, f->bus_id, f->frame.id);
}

// Sends the frames of the batch this device wants, in packets that fit its MTU (NimBLE would truncate a longer
// notification). A device whose earlier packets are still waiting misses the rest of this batch; the others are
// not held up by it.
static void send_to(const ble_receiver_t *r, const can_tagged_frame_t *batch, int count, uint8_t *buf, size_t max)
{
    for (int from = 0; from < count; ) {
        int before = from;
        size_t len = ble_stream_build(batch, count, &from, buf, max, want_frame, (void *)r);
        if (from == before) return;  // a frame larger than the MTU: drop the rest
        if (len && ble_server_notify_to(r, buf, len) != ESP_OK) return;
    }
}

// Task that bridges CAN frames to BLE notifications
static void can_to_ble_task(void *arg)
{
    (void)arg;
    static can_tagged_frame_t batch[BLE_BATCH_MAX_FRAMES];
    static uint8_t ble_buf[512];
    static ble_receiver_t rx[CAN_FILTER_SLOTS];

    while (true) {
        if (can_manager_receive(&batch[0], 100) != ESP_OK) {
            continue;
        }
        int count = 1;
        while (count < BLE_BATCH_MAX_FRAMES &&
               can_manager_receive(&batch[count], BLE_BATCH_TIMEOUT_MS) == ESP_OK) {
            count++;
        }

        // Each device that gets the stream, primary first, at its own MTU.
        size_t n = ble_server_receivers(rx, CAN_FILTER_SLOTS);
        for (size_t i = 0; i < n; i++) {
            size_t max = ble_server_payload(rx[i].conn);
            if (max > sizeof(ble_buf)) {
                max = sizeof(ble_buf);
            }
            if (max) {
                send_to(&rx[i], batch, count, ble_buf, max);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "DashKit firmware starting... (reset reason=%d)",
             (int)esp_reset_reason());

    // Initialize NVS (required for BLE)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erases the bond store: a bonded phone then hits the iOS stale-bond wall
        // and can't re-pair to this address on its own. Loud on purpose.
        ESP_LOGE(TAG, "NVS re-init (%s): erasing — all bonds will be lost",
                 esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(led_init());
    led_set_color(LED_COLOR_BLUE);

    ESP_ERROR_CHECK(can_manager_init());

    ESP_ERROR_CHECK(can_filter_init());

    dbc_init();

    ESP_ERROR_CHECK(automation_manager_init());

    ESP_ERROR_CHECK(vehicle_control_init());

    // CAN interface 0 (MCP2518FD)
    mcp251xfd_config_t can0_cfg = {
        .spi_host     = CAN0_SPI_HOST,
        .pin_mosi     = CAN0_PIN_MOSI,
        .pin_sclk     = CAN0_PIN_SCLK,
        .pin_miso     = CAN0_PIN_MISO,
        .pin_cs       = CAN0_PIN_CS,
        .pin_int      = CAN0_PIN_INT,
        .spi_clock_hz = CAN0_SPI_CLOCK_HZ,
        .osc_freq_hz  = CAN0_OSC_FREQ_HZ,
        .bitrate      = 500000,
        .bitrate_data = 0,
        .bus_id       = 0,
    };
    can_interface_t *can0 = mcp251xfd_create(&can0_cfg);
    if (!can0) {
        ESP_LOGE(TAG, "Failed to create CAN0 interface");
        led_set_color(LED_COLOR_RED);
        return;
    }
    ESP_ERROR_CHECK(can_manager_add_interface(can0));

    // CAN interface 1 (MCP2518FD, shares SPI2 bus with CAN0)
    mcp251xfd_config_t can1_cfg = {
        .spi_host     = CAN1_SPI_HOST,
        .pin_mosi     = CAN1_PIN_MOSI,
        .pin_sclk     = CAN1_PIN_SCLK,
        .pin_miso     = CAN1_PIN_MISO,
        .pin_cs       = CAN1_PIN_CS,
        .pin_int      = CAN1_PIN_INT,
        .spi_clock_hz = CAN1_SPI_CLOCK_HZ,
        .osc_freq_hz  = CAN1_OSC_FREQ_HZ,
        .bitrate      = 500000,
        .bitrate_data = 0,
        .bus_id       = 1,
    };
    can_interface_t *can1 = mcp251xfd_create(&can1_cfg);
    if (!can1) {
        ESP_LOGE(TAG, "Failed to create CAN1 interface");
        led_set_color(LED_COLOR_RED);
        return;
    }
    ESP_ERROR_CHECK(can_manager_add_interface(can1));

    // Start CAN
    err = can_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN manager start failed");
        led_set_color(LED_COLOR_RED);
        return;
    }

    // BLE
    ESP_ERROR_CHECK(ble_server_init());
    ESP_ERROR_CHECK(ble_ota_init());
    ESP_ERROR_CHECK(ble_appchan_init());
    ESP_ERROR_CHECK(ble_server_start());

#if defined(CONFIG_DASHKIT_TESLA_BLE)
    // Boot canary for the Tesla link: role state must be visible and a missing
    // key/link must not be silent. The app provisions the car (opcode 0x04)
    // and starts enrollment (0x01); until then the firmware only reports
    // "never enrolled".
    ESP_LOGI(TAG, "Tesla BLE: enabled (central=%d). Persistent VCSEC status "
                  "poll; enrollment is provisioned + started from the app.",
             CONFIG_BT_NIMBLE_ROLE_CENTRAL);
    ESP_ERROR_CHECK(tesla_pairing_init());
    ESP_ERROR_CHECK(tesla_ble_client_init());
#endif

    // Bridge task: CAN -> BLE
    xTaskCreatePinnedToCore(can_to_ble_task, "can2ble", 8192, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "DashKit firmware ready");

    // Commit this image once it has survived the boot window (OTA rollback).
    const esp_timer_create_args_t mv_args = {
        .callback = mark_app_valid_cb,
        .name = "ota_mark_valid",
    };
    esp_timer_handle_t mv_timer;
    ESP_ERROR_CHECK(esp_timer_create(&mv_args, &mv_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(mv_timer, MARK_VALID_DELAY_MS * 1000ULL));

    led_set_color(LED_COLOR_GREEN);
}
