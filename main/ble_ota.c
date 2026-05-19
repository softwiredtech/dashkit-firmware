#include "ble_ota.h"
#include "ble_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include <string.h>

static const char *TAG = "ble_ota";

// OTA control commands
#define OTA_CMD_BEGIN  0x01
#define OTA_CMD_ABORT  0x02

// OTA status codes
#define OTA_STATUS_PROGRESS  0x01
#define OTA_STATUS_DONE      0x02
#define OTA_STATUS_ERROR     0xFF

// Progress notification interval
#define OTA_PROGRESS_INTERVAL  4096

// OTA BLE UUIDs (CADA01xx series)
static const ble_uuid128_t s_ota_svc_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x00, 0x01, 0xDA, 0xCA
);

static const ble_uuid128_t s_ota_ctrl_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x01, 0x01, 0xDA, 0xCA
);

static const ble_uuid128_t s_ota_data_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x02, 0x01, 0xDA, 0xCA
);

static const ble_uuid128_t s_ota_status_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x03, 0x01, 0xDA, 0xCA
);

// OTA state
static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_update_partition = NULL;
static uint32_t s_fw_size = 0;
static uint32_t s_received = 0;
static bool s_ota_in_progress = false;
static uint32_t s_last_progress_notify = 0;

// BLE handles
static uint16_t s_ota_status_val_handle;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void send_status(uint8_t code, const uint8_t *payload, size_t payload_len)
{
    uint16_t conn = ble_server_get_conn_handle();
    if (conn == BLE_HS_CONN_HANDLE_NONE) return;

    uint8_t buf[6];
    buf[0] = code;
    size_t len = 1;
    if (payload && payload_len > 0 && payload_len <= sizeof(buf) - 1) {
        memcpy(&buf[1], payload, payload_len);
        len += payload_len;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (om) {
        ble_gatts_notify_custom(conn, s_ota_status_val_handle, om);
    }
}

static void send_progress(void)
{
    uint8_t payload[4];
    payload[0] = (s_received >>  0) & 0xFF;
    payload[1] = (s_received >>  8) & 0xFF;
    payload[2] = (s_received >> 16) & 0xFF;
    payload[3] = (s_received >> 24) & 0xFF;
    send_status(OTA_STATUS_PROGRESS, payload, 4);
}

static void send_error(uint8_t err_code)
{
    send_status(OTA_STATUS_ERROR, &err_code, 1);
}

static void ota_cleanup(void)
{
    if (s_ota_in_progress && s_ota_handle) {
        esp_ota_abort(s_ota_handle);
    }
    s_ota_handle = 0;
    s_update_partition = NULL;
    s_fw_size = 0;
    s_received = 0;
    s_ota_in_progress = false;
    s_last_progress_notify = 0;
}

static void reboot_timer_cb(void *arg)
{
    (void)arg;
    esp_restart();
}

// ---------------------------------------------------------------------------
// OTA command handling
// ---------------------------------------------------------------------------
static void handle_begin(const uint8_t *data, uint16_t len)
{
    if (len < 5) {
        ESP_LOGE(TAG, "Begin command too short");
        send_error(0x01);
        return;
    }

    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "OTA already in progress, aborting previous");
        ota_cleanup();
    }

    s_fw_size = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    if (s_fw_size == 0) {
        ESP_LOGE(TAG, "Invalid firmware size: 0");
        send_error(0x02);
        return;
    }

    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (!s_update_partition) {
        ESP_LOGE(TAG, "No OTA partition available");
        send_error(0x03);
        return;
    }

    if (s_fw_size > s_update_partition->size) {
        ESP_LOGE(TAG, "Firmware too large: %lu > %lu",
                 (unsigned long)s_fw_size, (unsigned long)s_update_partition->size);
        send_error(0x04);
        return;
    }

    esp_err_t err = esp_ota_begin(s_update_partition, s_fw_size, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_error(0x05);
        return;
    }

    s_received = 0;
    s_last_progress_notify = 0;
    s_ota_in_progress = true;

    ESP_LOGI(TAG, "OTA started: %lu bytes, partition '%s'",
             (unsigned long)s_fw_size, s_update_partition->label);

    // Send initial progress (0 bytes)
    send_progress();
}

static void handle_abort(void)
{
    if (!s_ota_in_progress) return;

    ESP_LOGW(TAG, "OTA aborted by client");
    ota_cleanup();
}

static void handle_data(const uint8_t *data, uint16_t len)
{
    if (!s_ota_in_progress) {
        send_error(0x10);
        return;
    }

    if (s_received + len > s_fw_size) {
        ESP_LOGE(TAG, "Received more data than expected");
        send_error(0x11);
        ota_cleanup();
        return;
    }

    esp_err_t err = esp_ota_write(s_ota_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        send_error(0x12);
        ota_cleanup();
        return;
    }

    s_received += len;

    // Send progress notification periodically
    if (s_received - s_last_progress_notify >= OTA_PROGRESS_INTERVAL) {
        send_progress();
        s_last_progress_notify = s_received;
    }

    // Check if transfer is complete
    if (s_received >= s_fw_size) {
        ESP_LOGI(TAG, "OTA transfer complete, validating...");

        err = esp_ota_end(s_ota_handle);
        s_ota_handle = 0;

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed (image invalid): %s", esp_err_to_name(err));
            send_error(0x13);
            ota_cleanup();
            return;
        }

        err = esp_ota_set_boot_partition(s_update_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
            send_error(0x14);
            ota_cleanup();
            return;
        }

        s_ota_in_progress = false;
        ESP_LOGI(TAG, "OTA successful, rebooting in 1s...");

        // Send completion notification
        send_status(OTA_STATUS_DONE, NULL, 0);

        // Reboot after a short delay to let the notification be sent
        const esp_timer_create_args_t timer_args = {
            .callback = reboot_timer_cb,
            .name = "ota_reboot",
        };
        esp_timer_handle_t timer;
        esp_timer_create(&timer_args, &timer);
        esp_timer_start_once(timer, 1000000);  // 1 second
    }
}

// ---------------------------------------------------------------------------
// GATT access callbacks
// ---------------------------------------------------------------------------
static int ota_ctrl_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t buf[5];
    uint16_t copy_len = len < sizeof(buf) ? len : sizeof(buf);
    os_mbuf_copydata(ctxt->om, 0, copy_len, buf);

    switch (buf[0]) {
    case OTA_CMD_BEGIN:
        handle_begin(buf, copy_len);
        break;
    case OTA_CMD_ABORT:
        handle_abort();
        break;
    default:
        ESP_LOGW(TAG, "Unknown OTA command: 0x%02x", buf[0]);
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    return 0;
}

static int ota_data_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0) return 0;

    // Read data from mbuf chain
    uint8_t *buf = malloc(len);
    if (!buf) {
        send_error(0x20);
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    os_mbuf_copydata(ctxt->om, 0, len, buf);
    handle_data(buf, len);
    free(buf);

    return 0;
}

static int ota_status_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    // Status is notify-only, return empty on read
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;
    }
    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

// ---------------------------------------------------------------------------
// GATT service definition
// ---------------------------------------------------------------------------
static const struct ble_gatt_svc_def s_ota_svc_def[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_ota_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Control: Write
                .uuid = &s_ota_ctrl_uuid.u,
                .access_cb = ota_ctrl_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                // Data: Write No Response
                .uuid = &s_ota_data_uuid.u,
                .access_cb = ota_data_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // Status: Notify + Read
                .uuid = &s_ota_status_uuid.u,
                .access_cb = ota_status_access,
                .val_handle = &s_ota_status_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
            },
            { 0 },  // Terminator
        },
    },
    { 0 },  // Terminator
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
const struct ble_gatt_svc_def *ble_ota_get_service_def(void)
{
    return s_ota_svc_def;
}

esp_err_t ble_ota_init(void)
{
    s_ota_in_progress = false;
    ESP_LOGI(TAG, "BLE OTA module initialized");
    return ESP_OK;
}

void ble_ota_on_disconnect(void)
{
    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "BLE disconnected during OTA, aborting");
        ota_cleanup();
    }
}
