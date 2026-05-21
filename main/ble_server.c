#include "ble_server.h"
#include "ble_ota.h"
#include "led.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <string.h>

// Provided by nimble/host/store/config — wires up the NVS-backed
// store_read_cb / store_write_cb / store_delete_cb so bonds persist
// across reboots. Without this, sm_bonding=1 only keeps keys in RAM.
extern void ble_store_config_init(void);

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

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_val_handle;
static bool     s_notifications_enabled = false;

// Pairing window state
static bool           s_pairing_allowed = false;
static TimerHandle_t  s_pairing_timer = NULL;
static TimerHandle_t  s_blink_timer = NULL;
static bool           s_blink_on = false;

static void pairing_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    s_pairing_allowed = false;
    if (s_blink_timer) {
        xTimerStop(s_blink_timer, 0);
    }
    led_set_color(LED_COLOR_GREEN);  // Restore solid green
    ESP_LOGI(TAG, "Pairing window closed");
}

static void blink_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    s_blink_on = !s_blink_on;
    led_set_color(s_blink_on ? LED_COLOR_GREEN : LED_COLOR_OFF);
}

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

            // Check if this peer is in our stored bonds. At BLE_GAP_EVENT_CONNECT
            // the link is brand new and unencrypted, so desc.sec_state.bonded is
            // always false — we must look the peer up in the persistent store.
            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(s_conn_handle, &desc);

            ble_addr_t bonded_peers[CONFIG_BT_NIMBLE_MAX_BONDS];
            int num_bonded = 0;
            ble_store_util_bonded_peers(bonded_peers, &num_bonded,
                                        CONFIG_BT_NIMBLE_MAX_BONDS);
            bool bonded = false;
            for (int i = 0; i < num_bonded; i++) {
                if (ble_addr_cmp(&bonded_peers[i], &desc.peer_id_addr) == 0) {
                    bonded = true;
                    break;
                }
            }

            if (!bonded && !s_pairing_allowed) {
                ESP_LOGW(TAG, "Rejecting unbonded peer (pairing window closed)");
                ble_gap_terminate(s_conn_handle, BLE_ERR_AUTH_FAIL);
                break;
            }

            // Do NOT touch MTU / PHY / GATT here. Wait until the link is
            // encrypted (BLE_GAP_EVENT_ENC_CHANGE). Issuing MTU exchange
            // while encryption is being set up races with the SMP exchange
            // and can crash the host on some configurations.
            if (!bonded) {
                // Unknown peer with pairing window open → drive pairing.
                // For already-bonded peers, Android auto-encrypts using the
                // stored LTK and we'd just race it by initiating here.
                ble_gap_security_initiate(s_conn_handle);
            }
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

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Encryption enabled (handle=%d)", event->enc_change.conn_handle);
            // Link is secure. The Android client drives the MTU exchange
            // (only the client may initiate it), so we don't call
            // ble_gattc_exchange_mtu here. We only request a faster PHY;
            // the central is free to accept or stay on 1M.
            ble_gap_set_prefered_le_phy(event->enc_change.conn_handle,
                                        BLE_GAP_LE_PHY_2M_MASK,
                                        BLE_GAP_LE_PHY_2M_MASK,
                                        BLE_GAP_LE_PHY_CODED_ANY);
        } else {
            ESP_LOGW(TAG, "Encryption failed: status=%d", event->enc_change.status);
            ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_AUTH_FAIL);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        if (!s_pairing_allowed) {
            ESP_LOGW(TAG, "Re-pairing rejected (pairing window closed)");
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        // Delete old bond and allow re-pairing
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        ESP_LOGI(TAG, "Allowing re-pairing");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        // Numeric Comparison: auto-accept on the ESP32 side.
        // The user confirms on the Android dialog, which is sufficient.
        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            struct ble_sm_io pk = {0};
            pk.action = BLE_SM_IOACT_NUMCMP;
            pk.numcmp_accept = 1;
            int rc = ble_sm_inject_io(event->passkey.conn_handle, &pk);
            ESP_LOGI(TAG, "Numeric Comparison auto-accepted (rc=%d)", rc);
        } else {
            ESP_LOGW(TAG, "Unexpected passkey action: %d", event->passkey.params.action);
        }
        break;
    }

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

    // Security manager: Numeric Comparison pairing with bonding.
    // Using DISP_YES_NO forces a confirmation dialog on Android,
    // which guarantees the bond is persisted.
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_YES_NO;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

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

    // Advertise that we can speak up to 512-byte ATT MTU. Android will
    // initiate the actual MTU exchange from its side (only the client may
    // initiate it), and we'll agree on the min of both sides' preferences.
    ble_att_set_preferred_mtu(512);

    // Install NVS-backed bond store (must be after nimble_port_init,
    // before nimble_port_run). This is what makes bonds survive reboots.
    ble_store_config_init();

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

void ble_server_enter_pairing_mode(uint32_t duration_sec)
{
    s_pairing_allowed = true;
    ESP_LOGI(TAG, "Pairing window open for %lu seconds", (unsigned long)duration_sec);

    // Create timers on first call
    if (!s_pairing_timer) {
        s_pairing_timer = xTimerCreate("pair_tmr", pdMS_TO_TICKS(duration_sec * 1000),
                                        pdFALSE, NULL, pairing_timer_cb);
        s_blink_timer = xTimerCreate("blink_tmr", pdMS_TO_TICKS(500),
                                      pdTRUE, NULL, blink_timer_cb);
    } else {
        xTimerChangePeriod(s_pairing_timer, pdMS_TO_TICKS(duration_sec * 1000), 0);
    }

    xTimerStart(s_pairing_timer, 0);

    // Blink green LED to indicate pairing mode
    s_blink_on = true;
    led_set_color(LED_COLOR_GREEN);
    xTimerStart(s_blink_timer, 0);
}
