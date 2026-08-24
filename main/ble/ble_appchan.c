#include "ble_appchan.h"
#include "ble_server.h"

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"

#if CONFIG_DASHKIT_TESLA_BLE
#include "tesla_ble_storage.h"
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
// Command writes (CADA0201): [opcode][value_lo][value_hi?] / [opcode][payload]
// ---------------------------------------------------------------------------
// The app-channel commands are the only enrollment triggers; they write on
// this dedicated characteristic, separate from CAN control (CADA0004).

// TESLA_CMD_PROVISION payload: [17B VIN][1B addr type][6B MAC in NimBLE's
// ble_addr_t.val order]. Stages the car; the app then sends 0x01 to enroll.
#if CONFIG_DASHKIT_TESLA_BLE
static esp_err_t app_provision(const uint8_t *p)
{
    char vin[18];
    tesla_car_addr_t addr;

    if (memchr(p, '\0', 17) != NULL) {
        return ESP_ERR_INVALID_ARG;   // VIN is 17 raw ASCII bytes, no NULs
    }
    memcpy(vin, p, 17);
    vin[17] = '\0';
    addr.type = p[17];
    memcpy(addr.val, &p[18], 6);
    return tesla_pairing_configure(vin, &addr);
}
#endif

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

    // Longest command is TESLA_CMD_PROVISION at 25 bytes; the rest are 3.
    uint8_t buf[TESLA_PROVISION_LEN];
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
    case TESLA_CMD_PROVISION:
        if (len != TESLA_PROVISION_LEN) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (tesla_storage_has_key()) {
            ESP_LOGW(TAG, "provision ignored: a key is already enrolled");
            return BLE_ATT_ERR_UNLIKELY;
        }
        err = app_provision(&buf[1]);
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

// Status characteristic: notify on change + serve the last frame so a fresh
// subscriber gets current state immediately.
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
    // Build the candidate frame first and drop it if nothing changed: the
    // pairing task re-reports staged/never-enrolled on a fast poll loop, and
    // notifying the identical frame at 5 Hz was pure spam. A changed frame is
    // always stored (so a later GATT read sees current state) but only
    // notified when a phone is connected.
    uint8_t frame[7];
    frame[0] = 0x01;               // frame version
    frame[1] = link_state;
    frame[2] = presence;
    frame[3] = lock;
    frame[4] = sleep;
    frame[5] = flags;
    frame[6] = (link_state == TESLA_LINK_ENROLLMENT_FAULT)
               ? fault_detail : TESLA_FAULT_NONE;

    if (memcmp(frame, s_last_frame, sizeof(frame)) == 0) {
        return;
    }
    memcpy(s_last_frame, frame, sizeof(frame));

    // The DashKit sits in the car trim and is driven by one phone at a time, so
    // the status notify targets only the single "active" connection (the one
    // subscribed to the CAN stream, ble_server's s_active_handle). Fan-out to
    // every subscribed phone is intentionally out of scope.
    uint16_t conn = ble_server_get_conn_handle();
    if (conn == BLE_HS_CONN_HANDLE_NONE || s_app_status_val_handle == 0) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_last_frame, sizeof(s_last_frame));
    if (om != NULL) {
        ble_gatts_notify_custom(conn, s_app_status_val_handle, om);
    }
}
