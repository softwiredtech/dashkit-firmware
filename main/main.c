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
#include "ble_ota.h"

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

// Build a DashPilot-compatible BLE packet from CAN frames.
// Format: [count][timestamp_us_LE32, bus, addr_LE32, len, data...]...
static size_t build_ble_packet(const can_tagged_frame_t *frames, int count, uint8_t *buf, size_t buf_size)
{
    size_t pos = 0;

    if (count <= 0 || pos >= buf_size) return 0;

    buf[pos++] = (uint8_t)count;

    for (int i = 0; i < count && pos < buf_size; i++) {
        const can_tagged_frame_t *f = &frames[i];
        uint8_t data_len = f->frame.dlc;
        if (data_len > 8) data_len = 8;  // DashPilot expects classic CAN (max 8)

        // Check we have room: timestamp(4) + bus(1) + addr(4) + len(1) + data(n)
        if (pos + 10 + data_len > buf_size) {
            buf[0] = (uint8_t)i;  // Adjust count to what we actually packed
            break;
        }

        // Timestamp in little-endian 32-bit (microseconds since boot)
        uint32_t ts = f->timestamp_us;
        buf[pos++] = (ts >>  0) & 0xFF;
        buf[pos++] = (ts >>  8) & 0xFF;
        buf[pos++] = (ts >> 16) & 0xFF;
        buf[pos++] = (ts >> 24) & 0xFF;

        buf[pos++] = f->bus_id;

        // Address in little-endian 32-bit
        uint32_t id = f->frame.id;
        buf[pos++] = (id >>  0) & 0xFF;
        buf[pos++] = (id >>  8) & 0xFF;
        buf[pos++] = (id >> 16) & 0xFF;
        buf[pos++] = (id >> 24) & 0xFF;

        buf[pos++] = data_len;
        memcpy(&buf[pos], f->frame.data, data_len);
        pos += data_len;
    }

    return pos;
}

// Task that bridges CAN frames to BLE notifications
static void can_to_ble_task(void *arg)
{
    (void)arg;
    static can_tagged_frame_t batch[BLE_BATCH_MAX_FRAMES];
    static uint8_t ble_buf[512];

    while (true) {
        int count = 0;

        if (can_manager_receive(&batch[0], 100) == ESP_OK) {
            count = 1;

            while (count < BLE_BATCH_MAX_FRAMES) {
                if (can_manager_receive(&batch[count], BLE_BATCH_TIMEOUT_MS) != ESP_OK) {
                    break;
                }
                count++;
            }

            // Send over BLE if connected
            if (ble_server_is_connected()) {
                size_t pkt_len = build_ble_packet(batch, count, ble_buf, sizeof(ble_buf));
                if (pkt_len > 0) {
                    ble_server_notify(ble_buf, pkt_len);
                }
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
    ESP_ERROR_CHECK(ble_server_start());

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
