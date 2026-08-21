#include "ble_appchan.h"
#include "ble_server.h"

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"

#if CONFIG_DASHKIT_TESLA_BLE
#include "tesla_pairing.h"
#endif

#include <string.h>

static const char *TAG = "ble_appchan";

// DashPilot app-channel UUIDs (CADA02xx).
// Service: CADA0200-...  Command (write): CADA0201-...  Status (notify): CADA0202-...
// Byte positions 13-14 of BLE_UUID128_INIT are little-endian, so the group nibbles
// are reversed: CADA0200 -> ...0x00,0x02,0xDA,0xCA at the end.
static const ble_uuid128_t s_app_svc_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x00, 0x02, 0xDA, 0xCA
);

static const ble_uuid128_t s_app_cmd_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x01, 0x02, 0xDA, 0xCA
);

static const ble_uuid128_t s_app_status_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x02, 0x02, 0xDA, 0xCA
);

static uint16_t s_app_status_val_handle;
// Last reported status frame; served on read so a subscribing app can learn
// current state immediately instead of waiting up to a reconnect backoff.
static uint8_t s_last_frame[7] = { 0x01, TESLA_LINK_NEVER_ENROLLED, 0xFF, 0xFF, 0xFF, 0, TESLA_FAULT_NONE };

// ---------------------------------------------------------------------------
// Command writes (CADA0201): [opcode][value_lo][value_hi?]
// ---------------------------------------------------------------------------
// The app-channel command is the ONLY trigger for Tesla enrollment. It writes
// on the dedicated service characteristic, so it is fully separate from the CAN
// control characteristic (CADA0004) and needs no help from vehicle_control.
static int app_cmd_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t buf[3];
    uint16_t copy_len = len < sizeof(buf) ? len : sizeof(buf);
    os_mbuf_copydata(ctxt->om, 0, copy_len, buf);
    uint8_t opcode = buf[0];

#if CONFIG_DASHKIT_TESLA_BLE
    esp_err_t err;
    switch (opcode) {
    case TESLA_CMD_START:
        err = tesla_pairing_start();
        break;
    case TESLA_CMD_RESET:
        err = tesla_pairing_reset();
        break;
    case TESLA_CMD_CANCEL:
        err = tesla_pairing_cancel();
        break;
    default:
        ESP_LOGW(TAG, "Unknown app-channel command: 0x%02x", opcode);
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "app-channel command 0x%02x failed: %s",
                 opcode, esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
    }
    ESP_LOGI(TAG, "app-channel command 0x%02x accepted", opcode);
#else
    ESP_LOGW(TAG, "app-channel command 0x%02x ignored (Tesla feature disabled)", opcode);
#endif

    return 0;
}

// Status characteristic: notify-on-change + read of the last frame, so a phone
// can learn current state immediately on subscribe (the pairing/client tasks
// push updates on change, but a fresh subscriber shouldn't wait for the next one).
static int app_status_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, s_last_frame, sizeof(s_last_frame)) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

// ---------------------------------------------------------------------------
// Service definition
// ---------------------------------------------------------------------------
static const struct ble_gatt_svc_def s_app_svc_def[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_app_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Command: write (encrypted), opcodes TESLA_CMD_*
                .uuid = &s_app_cmd_uuid.u,
                .access_cb = app_cmd_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
            },
            {
                // Status: notify + read (empty), encrypted subscribe via CCCD
                .uuid = &s_app_status_uuid.u,
                .access_cb = app_status_access,
                .val_handle = &s_app_status_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ
                       | BLE_GATT_CHR_F_READ_ENC,
            },
            { 0 },
        },
    },
    { 0 },
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
const struct ble_gatt_svc_def *ble_appchan_get_service_def(void)
{
    return s_app_svc_def;
}

esp_err_t ble_appchan_init(void)
{
    // NOTICE: must not reset s_app_status_val_handle here — NimBLE fills it
    // during ble_gatts_add_svcs() (called from ble_server_init), so this init
    // (which runs after) only logs. Reset would break status notifications.
    ESP_LOGI(TAG, "app-channel service initialized (CADA0200)");
    return ESP_OK;
}

void ble_appchan_report_status(uint8_t link_state, uint8_t presence, uint8_t lock,
                               uint8_t sleep, uint8_t flags, uint8_t fault_detail)
{
    uint16_t conn = ble_server_get_conn_handle();
    if (s_app_status_val_handle == 0) {
        return;
    }

    s_last_frame[0] = 0x01;               // frame version
    s_last_frame[1] = link_state;
    s_last_frame[2] = presence;
    s_last_frame[3] = lock;
    s_last_frame[4] = sleep;
    s_last_frame[5] = flags;
    s_last_frame[6] = (link_state == TESLA_LINK_ENROLLMENT_FAULT)
                      ? fault_detail : TESLA_FAULT_NONE;

    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_last_frame, sizeof(s_last_frame));
    if (om != NULL) {
        ble_gatts_notify_custom(conn, s_app_status_val_handle, om);
    }
}
