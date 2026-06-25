#include "board.h"
#include "led.h"
#include "can_interface.h"
#include "can_manager.h"
#include "can_filter.h"
#include "wiper_off.h"
#include "battery_preheat.h"
#include "three_finger.h"
#include "vehicle_control.h"
#include "mcp251xfd.h"
#include "ble_server.h"
#include "ble_ota.h"
#ifdef DASHKIT_SIM_MODE
#include "can_sim.h"
#endif

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "main";

// Maximum CAN frames per BLE notification packet.
// With ATT MTU 247 negotiated, payload room is ~244 bytes. Each frame is
// 1+4+1+4+1+8 = 19 bytes in the wire format, so up to ~12 frames fit per
// notify; allow some slack for shorter frames.
#define BLE_BATCH_MAX_FRAMES  16
// Maximum time to wait for a full batch before sending partial. Longer
// timeouts coalesce more frames per notify, lowering the notify rate
// NimBLE has to push through the connection interval. 20ms keeps UI
// latency snappy while letting ~7 frames pack per BLE packet at the
// realistic ~335 fps CAN load.
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
    // Keep these out of the task stack: each can_tagged_frame_t is ~80 bytes
    // (CAN FD data field), so a 16-frame batch + 512B BLE buffer is ~1.8KB
    // of locals before any function call. Combined with NimBLE's deep call
    // chain from ble_server_notify, that overflows even an 8KB stack under
    // heavy load.
    static can_tagged_frame_t batch[BLE_BATCH_MAX_FRAMES];
    static uint8_t ble_buf[512];

    while (true) {
        int count = 0;

        // Wait for at least one frame. Filtering happens at the CAN driver
        // callback, so anything we get here is already destined for BLE.
        if (can_manager_receive(&batch[0], 100) == ESP_OK) {
            count = 1;

            // Try to fill the batch, flushing as soon as matching traffic
            // pauses for BLE_BATCH_TIMEOUT_MS.
            while (count < BLE_BATCH_MAX_FRAMES) {
                if (can_manager_receive(&batch[count], BLE_BATCH_TIMEOUT_MS) != ESP_OK) {
                    break;
                }
                count++;
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

    // CAN filter (configured by BLE client; empty = forward all)
    ESP_ERROR_CHECK(can_filter_init());

    // Auto wiper-off automation
    ESP_ERROR_CHECK(wiper_off_init());

    // Battery preheat injector (faked UI_tripPlanning), toggled over BLE
    ESP_ERROR_CHECK(battery_preheat_init());

    // Three-finger infotainment tap -> bound action (set over BLE)
    ESP_ERROR_CHECK(three_finger_init());

    // Vehicle control commands (BLE -> Tesla UI_* control frames)
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

#ifdef DASHKIT_SIM_MODE
    // Simulator task: emits DashPilot's filtered Tesla message set
    can_sim_start();
#endif

    ESP_LOGI(TAG, "DashKit firmware ready");

    led_set_color(LED_COLOR_GREEN);
}
