#include "ble_server.h"
#include "ble_ota.h"
#include "can_filter.h"
#include "can_manager.h"
#include "can_interface.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "ble";

#define DEVICE_NAME "DashKit"

// DashPilot BLE UUIDs
// Service:        CADA0000-CA00-B1E0-B0D6-C000AA0100A1
// Characteristic: CADA0001-CA00-B1E0-B0D6-C000AA0100A1
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x00, 0x00, 0xDA, 0xCA
);

static const ble_uuid128_t s_chr_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x01, 0x00, 0xDA, 0xCA
);

// CAN filter characteristic (write):
//   CADA0002-CA00-B1E0-B0D6-C000AA0100A1
// Wire format: [bus][num_addrs][addr_LE32]*  repeated per bus.
// Empty write clears the filter.
static const ble_uuid128_t s_filter_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x02, 0x00, 0xDA, 0xCA
);

// CAN TX characteristic (write):
//   CADA0003-CA00-B1E0-B0D6-C000AA0100A1
// Wire format (one CAN frame per write):
//   [bus][flags][addr_LE32][len][data...]
//   flags: bit0 = extended ID (29-bit), bit1 = CAN FD, bit2 = bit rate switch
//   len: number of data bytes that follow (<= 8 for classic, <= 64 for FD)
static const ble_uuid128_t s_tx_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x03, 0x00, 0xDA, 0xCA
);

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_val_handle;
static bool     s_notifications_enabled = false;

// ---------------------------------------------------------------------------
// GAP event handling
// ---------------------------------------------------------------------------
static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected (handle=%d)", s_conn_handle);
            // Request higher MTU for larger CAN frame batches
            ble_att_set_preferred_mtu(512);
            ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);
            // Request 2M PHY for higher throughput
            ble_gap_set_prefered_le_phy(s_conn_handle,
                                        BLE_GAP_LE_PHY_2M_MASK,
                                        BLE_GAP_LE_PHY_2M_MASK,
                                        BLE_GAP_LE_PHY_CODED_ANY);
        } else {
            ESP_LOGW(TAG, "Connection failed, status=%d", event->connect.status);
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected (reason=0x%x)", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notifications_enabled = false;
        ble_ota_on_disconnect();
        // Drop the filter so the next client starts in pass-through mode.
        can_filter_set(NULL, 0);
        start_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_chr_val_handle) {
            s_notifications_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Notifications %s", s_notifications_enabled ? "enabled" : "disabled");
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE: {
        uint8_t tx_phy = 0, rx_phy = 0;
        ble_gap_read_le_phy(event->phy_updated.conn_handle, &tx_phy, &rx_phy);
        ESP_LOGI(TAG, "PHY updated: TX=%sM, RX=%sM",
                 tx_phy == BLE_GAP_LE_PHY_2M ? "2" : "1",
                 rx_phy == BLE_GAP_LE_PHY_2M ? "2" : "1");
        break;
    }

    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 0x20,   // 20ms
        .itvl_max = 0x40,   // 40ms
    };

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    ble_gap_adv_set_fields(&fields);

    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, gap_event_handler, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Advertising start failed: %d", rc);
    }
}

// ---------------------------------------------------------------------------
// GATT access callback
// ---------------------------------------------------------------------------
static int gatt_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    // This characteristic is notify-only, no read/write from client
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Return empty for reads
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// Write callback for the CAN filter characteristic. Drains the mbuf chain
// into a flat buffer and hands it to can_filter_set().
static int gatt_filter_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t buf[256];
    if (len > sizeof(buf)) {
        ESP_LOGW(TAG, "Filter write too large (%u bytes)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGI(TAG, "CAN filter write received (%u bytes)", out_len);
    esp_err_t err = can_filter_set(buf, out_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "can_filter_set failed: %s", esp_err_to_name(err));
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    return 0;
}

// Write callback for the CAN TX characteristic. Parses one CAN frame from
// the wire format and dispatches it to the matching CAN interface.
static int gatt_tx_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Header: bus(1) + flags(1) + addr(4) + len(1) = 7 bytes
    uint16_t in_len = OS_MBUF_PKTLEN(ctxt->om);
    if (in_len < 7) {
        ESP_LOGW(TAG, "TX write too short (%u bytes)", in_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t buf[7 + CAN_MAX_DATA_LEN];
    if (in_len > sizeof(buf)) {
        ESP_LOGW(TAG, "TX write too large (%u bytes)", in_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t  bus    = buf[0];
    uint8_t  flags  = buf[1];
    uint32_t addr   = (uint32_t)buf[2]
                    | ((uint32_t)buf[3] << 8)
                    | ((uint32_t)buf[4] << 16)
                    | ((uint32_t)buf[5] << 24);
    uint8_t  dlc    = buf[6];

    if (7 + dlc > out_len) {
        ESP_LOGW(TAG, "TX write: dlc=%u exceeds payload (%u bytes)", dlc, out_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (dlc > CAN_MAX_DATA_LEN) {
        ESP_LOGW(TAG, "TX write: dlc=%u exceeds CAN_MAX_DATA_LEN", dlc);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    can_frame_t frame = {
        .id       = addr,
        .dlc      = dlc,
        .extended = (flags & 0x01) != 0,
        .fd       = (flags & 0x02) != 0,
        .brs      = (flags & 0x04) != 0,
    };
    if (dlc > 0) {
        memcpy(frame.data, &buf[7], dlc);
    }

    esp_err_t err = can_manager_send(bus, &frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "can_manager_send(bus=%u, id=0x%lX) failed: %s",
                 bus, (unsigned long)addr, esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// GATT service definition
// ---------------------------------------------------------------------------
// CAN service only — OTA service is added dynamically in ble_server_init
static const struct ble_gatt_svc_def s_can_svc_def[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_chr_uuid.u,
                .access_cb = gatt_chr_access,
                .val_handle = &s_chr_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &s_filter_uuid.u,
                .access_cb = gatt_filter_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &s_tx_uuid.u,
                .access_cb = gatt_tx_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            { 0 },  // Terminator
        },
    },
    { 0 },  // Terminator
};

// Combined GATT services: CAN + OTA + terminator
static struct ble_gatt_svc_def s_gatt_svcs[3];

// ---------------------------------------------------------------------------
// NimBLE host sync callback
// ---------------------------------------------------------------------------
static void on_sync(void)
{
    // Use best available address
    ble_hs_util_ensure_addr(0);
    start_advertising();
    ESP_LOGI(TAG, "BLE host synced, advertising as \"%s\"", DEVICE_NAME);
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();  // Returns only when nimble_port_stop() is called
    nimble_port_freertos_deinit();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
esp_err_t ble_server_init(void)
{
    int rc;

    rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return ESP_FAIL;
    }

    // Configure NimBLE host
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_sc = 0;

    // Build combined GATT service table: CAN + OTA
    const struct ble_gatt_svc_def *ota_svc = ble_ota_get_service_def();
    s_gatt_svcs[0] = s_can_svc_def[0];  // CAN service
    s_gatt_svcs[1] = ota_svc[0];        // OTA service
    memset(&s_gatt_svcs[2], 0, sizeof(s_gatt_svcs[2]));  // Terminator

    // Register GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    // Set device name
    ble_svc_gap_device_name_set(DEVICE_NAME);

    ESP_LOGI(TAG, "BLE GATT server initialized");
    return ESP_OK;
}

esp_err_t ble_server_start(void)
{
    nimble_port_freertos_init(nimble_host_task);
    return ESP_OK;
}

esp_err_t ble_server_notify(const uint8_t *data, size_t len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notifications_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    // Pause CAN notifications during OTA to avoid BLE congestion
    if (ble_ota_is_in_progress()) {
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_chr_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Notify failed: %d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool ble_server_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_notifications_enabled;
}

uint16_t ble_server_get_conn_handle(void)
{
    return s_conn_handle;
}
