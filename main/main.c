#include "board.h"
#include "led.h"
#include "can_interface.h"
#include "can_manager.h"
#include "mcp251xfd.h"
#include "ble_server.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "main";

// Maximum CAN frames per BLE notification packet
#define BLE_BATCH_MAX_FRAMES  16
// Maximum time to wait for a full batch before sending partial
#define BLE_BATCH_TIMEOUT_MS  5

// Build a DashPilot-compatible BLE packet from CAN frames.
// Format: [count][bus, addr_LE32, len, data...]...
static size_t build_ble_packet(const can_tagged_frame_t *frames, int count, uint8_t *buf, size_t buf_size)
{
    size_t pos = 0;

    if (count <= 0 || pos >= buf_size) return 0;

    buf[pos++] = (uint8_t)count;

    for (int i = 0; i < count && pos < buf_size; i++) {
        const can_tagged_frame_t *f = &frames[i];
        uint8_t data_len = f->frame.dlc;
        if (data_len > 8) data_len = 8;  // DashPilot expects classic CAN (max 8)

        // Check we have room: bus(1) + addr(4) + len(1) + data(n)
        if (pos + 6 + data_len > buf_size) {
            buf[0] = (uint8_t)i;  // Adjust count to what we actually packed
            break;
        }

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
    can_tagged_frame_t batch[BLE_BATCH_MAX_FRAMES];
    uint8_t ble_buf[512];

    while (true) {
        int count = 0;

        // Wait for at least one frame
        if (can_manager_receive(&batch[0], 100) == ESP_OK) {
            count = 1;

            // Try to fill the batch without blocking long
            while (count < BLE_BATCH_MAX_FRAMES) {
                if (can_manager_receive(&batch[count], BLE_BATCH_TIMEOUT_MS) == ESP_OK) {
                    count++;
                } else {
                    break;
                }
            }

#ifdef CONFIG_DASHKIT_DEBUG_LOG
            // Log frames to serial
            for (int i = 0; i < count; i++) {
                const can_tagged_frame_t *f = &batch[i];
                uint8_t len = f->frame.dlc > 8 ? 8 : f->frame.dlc;
                char hex[8 * 3 + 1] = {0};
                for (int j = 0; j < len; j++) {
                    sprintf(hex + j * 3, "%02X ", f->frame.data[j]);
                }
                ESP_LOGI(TAG, "CAN%d 0x%03lX [%d] %s",
                         f->bus_id, (unsigned long)f->frame.id, len, hex);
            }
#endif

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
    ESP_LOGI(TAG, "DashKit firmware starting...");

    // Initialize NVS (required for BLE)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // LED
    ESP_ERROR_CHECK(led_init());
    led_set_color(LED_COLOR_BLUE);  // Blue = booting

    // CAN manager
    ESP_ERROR_CHECK(can_manager_init());

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
        .bus_id       = 1,
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
        .bus_id       = 2,
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
    ESP_ERROR_CHECK(ble_server_start());

    // Bridge task: CAN -> BLE
    xTaskCreatePinnedToCore(can_to_ble_task, "can2ble", 4096, NULL, 5, NULL, 0);

    led_set_color(LED_COLOR_GREEN);  // Green = running
    ESP_LOGI(TAG, "DashKit firmware ready");
}
