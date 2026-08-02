#include "ble_server.h"
#include "ble_ota.h"
#include "can_filter.h"
#include "can_manager.h"
#include "can_interface.h"
#include "vehicle_control.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

// Provided by the NimBLE store/config component (NVS-backed bond storage).
void ble_store_config_init(void);

// Sets the ATT permissions for every CCCD (the descriptor a client writes to
// subscribe). Declared only in NimBLE's private ble_gatt_priv.h.
void ble_gatts_set_clt_cfg_perm_flags(uint8_t flags);

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

// Vehicle control characteristic (write):
//   CADA0004-CA00-B1E0-B0D6-C000AA0100A1
// Wire format (one command per write):
//   [opcode][value_lo][value_hi?]
//   opcode: vehicle_control_opcode_t (see vehicle_control.h)
//   value:  1 or 2 bytes, little-endian raw signal value
static const ble_uuid128_t s_control_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x04, 0x00, 0xDA, 0xCA
);

// Firmware version characteristic (read):
//   CADA0005-CA00-B1E0-B0D6-C000AA0100A1
// Returns the running firmware version string (esp_app_desc_t.version, set
// from version.txt at build time). Plain read (no encryption) so the app can
// query the version even before bonding, to decide whether an OTA is needed.
static const ble_uuid128_t s_version_uuid = BLE_UUID128_INIT(
    0xA1, 0x00, 0x01, 0xAA, 0x00, 0xC0, 0xD6, 0xB0,
    0xE0, 0xB1, 0x00, 0xCA, 0x05, 0x00, 0xDA, 0xCA
);

// Exactly one phone may be connected at a time (CONFIG_BT_NIMBLE_MAX_CONNECTIONS
// is 1). Several phones may still be *bonded*; they just take turns connecting.
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool     s_notify_enabled = false;   // CAN notifications subscribed
static bool     s_peer_was_bonded = false;  // peer had a stored bond at CONNECT
static bool     s_encrypted = false;        // link encryption established
static uint16_t s_chr_val_handle;

// A connection that has not established encryption within this time is dropped,
// so an unknown device cannot squat on the only connection slot.
#define SEC_TIMEOUT_S 30
static esp_timer_handle_t s_sec_timer = NULL;

// Pairing mode gates the creation of NEW bonds. When false, a device that is
// not already bonded cannot pair (its fresh bond is torn down at ENC_CHANGE,
// and re-pair attempts by bonded peers are ignored). It is true only when no
// bonds exist yet (first-time setup) or during a window opened on command by an
// already-paired device.
#define PAIRING_WINDOW_S   120
static bool               s_pairing_mode = false;
static esp_timer_handle_t s_pairing_timer = NULL;

// ---------------------------------------------------------------------------
// GAP event handling
// ---------------------------------------------------------------------------
static void start_advertising(void);

// Number of bonds currently persisted in the NVS key store.
static int bond_count(void)
{
    int count = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count);
    return count;
}

// True if this peer identity address has a bond in the NVS key store. The
// controller resolves RPAs of bonded peers before CONNECT, so peer_id_addr is
// the identity address whenever we hold the peer's IRK.
static bool peer_is_bonded(const ble_addr_t *peer_id_addr)
{
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = 0;
    if (ble_store_util_bonded_peers(peers, &n, CONFIG_BT_NIMBLE_MAX_BONDS) != 0) {
        return false;
    }
    for (int i = 0; i < n; i++) {
        if (ble_addr_cmp(&peers[i], peer_id_addr) == 0) {
            return true;
        }
    }
    return false;
}

// Fires when a connection sits unencrypted for SEC_TIMEOUT_S.
static void sec_timeout_cb(void *arg)
{
    (void)arg;
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE && !s_encrypted) {
        ESP_LOGW(TAG, "No encryption within %d s; dropping handle %d",
                 SEC_TIMEOUT_S, s_conn_handle);
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

// Timer fires when a pairing window elapses without a new device pairing.
static void pairing_timer_cb(void *arg)
{
    (void)arg;
    // Stay pairable only if there are still no bonds (first-time setup).
    s_pairing_mode = (bond_count() == 0);
    ESP_LOGI(TAG, "Pairing window elapsed (pairing_mode=%d)", s_pairing_mode);
}

static void open_pairing_window(void)
{
    if (!s_pairing_timer) {
        const esp_timer_create_args_t args = {
            .callback = pairing_timer_cb,
            .name = "pair_win",
        };
        esp_timer_create(&args, &s_pairing_timer);
    }
    esp_timer_stop(s_pairing_timer);  // harmless if not running
    esp_timer_start_once(s_pairing_timer, (uint64_t)PAIRING_WINDOW_S * 1000000ULL);
    s_pairing_mode = true;
    ESP_LOGI(TAG, "Pairing window opened (%d s)", PAIRING_WINDOW_S);
}

static void close_pairing_window(void)
{
    if (s_pairing_timer) {
        esp_timer_stop(s_pairing_timer);
    }
    s_pairing_mode = (bond_count() == 0);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Connection failed, status=%d", event->connect.status);
            start_advertising();
            break;
        }
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            // Should not happen with MAX_CONNECTIONS=1; drop the newcomer.
            ESP_LOGW(TAG, "Already connected; dropping handle %d",
                     event->connect.conn_handle);
            ble_gap_terminate(event->connect.conn_handle,
                              BLE_ERR_REM_USER_CONN_TERM);
            break;
        }

        // Note: bonded phones are NOT turned away while a pairing window is
        // open. The DashPilot apps pause their auto-reconnect after sending the
        // enter-pairing command instead, keeping the slot free for the new
        // device without firmware-side connection juggling.
        struct ble_gap_conn_desc desc;
        bool bonded = ble_gap_conn_find(event->connect.conn_handle, &desc) == 0
                   && peer_is_bonded(&desc.peer_id_addr);

        s_conn_handle = event->connect.conn_handle;
        s_peer_was_bonded = bonded;
        s_encrypted = false;
        s_notify_enabled = false;
        ESP_LOGI(TAG, "Connected (handle=%d, peer_bonded=%d, bonds=%d)",
                 s_conn_handle, bonded, bond_count());
        ble_att_set_preferred_mtu(512);
        ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);
        esp_timer_stop(s_sec_timer);  // harmless if not running
        esp_timer_start_once(s_sec_timer, (uint64_t)SEC_TIMEOUT_S * 1000000ULL);
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected (handle=%d, reason=0x%x)",
                 event->disconnect.conn.conn_handle, event->disconnect.reason);
        if (event->disconnect.conn.conn_handle == s_conn_handle) {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_notify_enabled = false;
            s_encrypted = false;
            s_peer_was_bonded = false;
            esp_timer_stop(s_sec_timer);
            ble_ota_on_disconnect();
            // Reset the CAN filter to pass-through for the next client.
            can_filter_set(NULL, 0);
        }
        start_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_chr_val_handle &&
            event->subscribe.conn_handle == s_conn_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Notifications %s (handle=%d)",
                     s_notify_enabled ? "enabled" : "disabled",
                     event->subscribe.conn_handle);
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

    case BLE_GAP_EVENT_ENC_CHANGE: {
        uint16_t ch = event->enc_change.conn_handle;
        ESP_LOGI(TAG, "Encryption change: status=%d (handle=%d)",
                 event->enc_change.status, ch);
        if (event->enc_change.status != 0) {
            // Encryption/pairing failed (e.g. a phone holding a stale bond we no
            // longer have, or a pairing we refused). Drop only this link — never
            // reboot here, or any stranger could restart the CAN bridge at will.
            ESP_LOGW(TAG, "Encryption failed (status=%d); terminating handle %d",
                     event->enc_change.status, ch);
            ble_gap_terminate(ch, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }
        if (ch != s_conn_handle) {
            break;
        }

        // The peer had no stored bond when it connected, so this encryption can
        // only come from a freshly-created bond.
        if (!s_peer_was_bonded && !s_pairing_mode) {
            // Unsolicited pairing: a new device bonded while we are locked down.
            // Delete the fresh bond and drop the link before it can use the
            // encryption-protected characteristics.
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(ch, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            ESP_LOGW(TAG, "Rejecting pairing (handle=%d): not in pairing mode", ch);
            ble_gap_terminate(ch, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }

        s_encrypted = true;
        esp_timer_stop(s_sec_timer);

        if (!s_peer_was_bonded) {
            // A new device paired during an open window: close it (one device
            // per window).
            ESP_LOGI(TAG, "New device paired; closing pairing window");
            s_peer_was_bonded = true;
            close_pairing_window();
        }

        ble_gap_set_prefered_le_phy(ch, BLE_GAP_LE_PHY_2M_MASK,
                                    BLE_GAP_LE_PHY_2M_MASK,
                                    BLE_GAP_LE_PHY_CODED_ANY);
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        if (!s_pairing_mode) {
            // An already-bonded peer is trying to pair again while we are locked
            // down. Ignore it: keep the existing bond, reject the re-pair.
            ESP_LOGW(TAG, "Repeat pairing ignored: not in pairing mode");
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        // In pairing mode: delete our stale bond and let the fresh pairing run.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            int drc = ble_store_util_delete_peer(&desc.peer_id_addr);
            ESP_LOGW(TAG, "Repeat pairing: deleted stale bond (rc=%d), retrying", drc);
        } else {
            ESP_LOGW(TAG, "Repeat pairing: conn desc not found");
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(TAG, "Passkey action: %d (Just Works, no input required)",
                 event->passkey.params.action);
        break;

    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    // The only connection slot is taken; advertise again once it frees up.
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

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

void ble_server_enter_pairing_mode(void)
{
    open_pairing_window();
    // The new device needs the only connection slot. Drop the current
    // connection — normally the phone that sent the command — so the new device
    // can connect within the window. The requesting app pauses its
    // auto-reconnect for the window duration so it doesn't take the slot back.
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "Pairing mode: dropping handle %d to free the slot",
                 s_conn_handle);
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        // The DISCONNECT handler restarts advertising.
        return;
    }
    start_advertising();
}

void ble_server_delete_all_bonds(void)
{
    int rc = ble_store_clear();
    ESP_LOGW(TAG, "Deleted all BLE bonds (rc=%d)", rc);
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

// Read callback for the firmware version characteristic. Returns the running
// image's version string (from esp_app_desc_t, populated from version.txt).
static int gatt_version_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const esp_app_desc_t *desc = esp_app_get_description();
    int rc = os_mbuf_append(ctxt->om, desc->version, strlen(desc->version));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Write callback for the vehicle control characteristic. Parses a high-level
// [opcode][value] command and hands it to the vehicle control subsystem, which
// encodes and transmits the matching Tesla CAN frame.
static int gatt_control_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Format: opcode(1) + value(1..2, little-endian)
    uint16_t in_len = OS_MBUF_PKTLEN(ctxt->om);
    if (in_len < 2 || in_len > 3) {
        ESP_LOGW(TAG, "control write bad length (%u bytes)", in_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t buf[3];
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t  opcode = buf[0];
    uint16_t value  = buf[1];
    if (out_len >= 3) {
        value |= (uint16_t)buf[2] << 8;
    }

    esp_err_t err = vehicle_control_submit(opcode, value);
    if (err == ESP_ERR_INVALID_ARG) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "vehicle_control_submit(0x%02X) failed: %s",
                 opcode, esp_err_to_name(err));
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
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ
                       | BLE_GATT_CHR_F_READ_ENC,
            },
            {
                .uuid = &s_filter_uuid.u,
                .access_cb = gatt_filter_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
            },
            {
                .uuid = &s_control_uuid.u,
                .access_cb = gatt_control_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
            },
            {
                .uuid = &s_version_uuid.u,
                .access_cb = gatt_version_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 },
        },
    },
    { 0 },
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
    // With no bonds yet we are in first-time setup: stay pairable. Once a device
    // is bonded, lock down until a paired device opens a pairing window.
    s_pairing_mode = (bond_count() == 0);
    start_advertising();
    ESP_LOGI(TAG, "BLE host synced, advertising as \"%s\" (pairing_mode=%d)",
             DEVICE_NAME, s_pairing_mode);
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

    const esp_timer_create_args_t sec_timer_args = {
        .callback = sec_timeout_cb,
        .name = "ble_sec",
    };
    esp_timer_create(&sec_timer_args, &s_sec_timer);

    rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return ESP_FAIL;
    }

    // Configure NimBLE host
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;

    // "Just Works" pairing + bonding
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_sc = 1;  // LE Secure Connections
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    // On key-store overflow, evict the oldest entry instead of failing the
    // pairing (matters when a new phone pairs with MAX_BONDS already stored).
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Wire up the NVS-backed key store so bonds survive a reboot.
    ble_store_config_init();

    // Build combined GATT service table: CAN + OTA
    const struct ble_gatt_svc_def *ota_svc = ble_ota_get_service_def();
    s_gatt_svcs[0] = s_can_svc_def[0];  // CAN service
    s_gatt_svcs[1] = ota_svc[0];        // OTA service
    memset(&s_gatt_svcs[2], 0, sizeof(s_gatt_svcs[2]));  // Terminator

    // Register GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Require encryption to subscribe: the char's READ_ENC gates client reads,
    // not server notifications, so without this the CAN stream reaches unpaired
    // devices. Must run before ble_gatts_add_svcs() registers the CCCDs.
    ble_gatts_set_clt_cfg_perm_flags(BLE_ATT_F_READ | BLE_ATT_F_READ_ENC |
                                     BLE_ATT_F_WRITE | BLE_ATT_F_WRITE_ENC);

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
    // Pause CAN notifications during OTA to avoid BLE congestion
    if (ble_ota_is_in_progress()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Only stream to an encrypted, subscribed link (belt-and-suspenders with
    // the CCCD encryption requirement).
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notify_enabled || !s_encrypted) {
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        return ESP_ERR_NO_MEM;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_chr_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Notify failed (handle=%d): %d", s_conn_handle, rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool ble_server_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_notify_enabled;
}

uint16_t ble_server_get_conn_handle(void)
{
    return s_conn_handle;
}
