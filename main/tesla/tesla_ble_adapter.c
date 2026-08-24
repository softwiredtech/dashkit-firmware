/*
 * Tesla BLE adapter — NimBLE central connection to the vehicle-command GATT
 * service.
 *
 * The connect callback and GATT discovery flow never feed the peripheral
 * server's slot table in main/ble/ble_server.c. There is deliberately no
 * observer/scan path here: the car's address is provisioned by the app
 * (app-channel opcode 0x04) or resumed from NVS.
 */

#include "tesla_ble_adapter.h"

#include "protobuf_build.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"

#include <string.h>

static const char *TAG = "tesla_ble";

// ---- vehicle-command GATT UUIDs (LE byte order) ----
// Service:  00000211-b2d1-43f0-9b88-960cebf8b91e
// Write:    00000212-b2d1-43f0-9b88-960cebf8b91e  (write with response)
// Indicate: 00000213-b2d1-43f0-9b88-960cebf8b91e
#define TESLA_CONNECT_TIMEOUT_MS 20000
// Includes the two-byte vehicle-command length prefix. Keep this derived from
// the public frame bound so the adapter and its consumers cannot disagree.
#define TESLA_RX_BUF (TESLA_RX_FRAME_MAX + 2)

// ---------------------------------------------------------------------------
// Central-connection state
// ---------------------------------------------------------------------------
typedef enum {
    ST_IDLE,
    ST_CONNECTING,
    ST_DISCOVERING,
    ST_SUBSCRIBING,
    ST_READY,
    ST_DISCONNECTING,
} central_state_t;

enum {
    GATT_OWNER_NONE = 0,
    GATT_OWNER_TX,
    GATT_OWNER_KEEPALIVE,
};

static struct {
    central_state_t state;
    uint16_t conn_handle;
    uint16_t svc_start, svc_end;
    uint16_t tx_handle, rx_handle, rx_def_handle, rx_dsc_end, rx_cccd_handle;
    uint8_t tx_properties;
    SemaphoreHandle_t ready_sem;   // binary: posted when READY or failed
    SemaphoreHandle_t disconnect_sem;
    SemaphoreHandle_t gatt_sem;    // serializes TX and keepalive procedures
    SemaphoreHandle_t tx_done_sem; // write-with-response chunk completion
    int tx_status;
    bool connect_ok;
    bool keepalive_pending;
    uint8_t gatt_owner;
    bool tx_abort;
    uint32_t generation;
    // RX framing + dispatch
    uint8_t rx_buf[TESLA_RX_BUF];
    size_t  rx_len;
    tesla_ble_rx_fn_t rx_cb;
    void   *rx_arg;
} s_central;

// NimBLE invokes the notification callback on the host task. Keep the largest
// ATT chunk out of that task's stack; rx_ingest() copies it into s_central's
// framing buffer before dispatching complete vehicle frames.
static uint8_t s_notify_buf[TESLA_RX_BUF];

// ---------------------------------------------------------------------------
// Central connection
// ---------------------------------------------------------------------------
#define SVC_UUID_INIT \
    0x1e, 0xb9, 0xf8, 0xeb, 0x0c, 0x96, 0x88, 0x9b, \
    0xf0, 0x43, 0xd1, 0xb2, 0x11, 0x02, 0x00, 0x00
#define WRITE_UUID_INIT \
    0x1e, 0xb9, 0xf8, 0xeb, 0x0c, 0x96, 0x88, 0x9b, \
    0xf0, 0x43, 0xd1, 0xb2, 0x12, 0x02, 0x00, 0x00
#define INDICATE_UUID_INIT \
    0x1e, 0xb9, 0xf8, 0xeb, 0x0c, 0x96, 0x88, 0x9b, \
    0xf0, 0x43, 0xd1, 0xb2, 0x13, 0x02, 0x00, 0x00

static const ble_uuid128_t s_svc_uuid     = BLE_UUID128_INIT(SVC_UUID_INIT);
static const ble_uuid128_t s_write_uuid   = BLE_UUID128_INIT(WRITE_UUID_INIT);
static const ble_uuid128_t s_indicate_uuid = BLE_UUID128_INIT(INDICATE_UUID_INIT);

// Notify the waiting task that the connect sequence finished. Runs on the
// NimBLE host task (a normal task), not an ISR.
static bool generation_current(void *arg)
{
    return (uint32_t)(uintptr_t)arg == s_central.generation;
}

static bool connection_current(uint16_t conn_handle, void *arg)
{
    return generation_current(arg) &&
           s_central.conn_handle == conn_handle &&
           s_central.state != ST_IDLE &&
           s_central.state != ST_DISCONNECTING;
}

static void central_done(uint16_t conn_handle, void *arg, bool ok)
{
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale central completion handle=%u generation=%lu",
                 conn_handle, (unsigned long)(uintptr_t)arg);
        return;
    }
    s_central.connect_ok = ok;
    if (s_central.ready_sem != NULL) {
        xSemaphoreGive(s_central.ready_sem);
    }
}

static void central_done_connect(void *arg, bool ok)
{
    if (!generation_current(arg) || s_central.state != ST_CONNECTING) {
        ESP_LOGW(TAG, "ignoring stale connect completion generation=%lu",
                 (unsigned long)(uintptr_t)arg);
        return;
    }
    s_central.connect_ok = ok;
    if (s_central.ready_sem != NULL) {
        xSemaphoreGive(s_central.ready_sem);
    }
}

// Feed a complete frame (payload, framing stripped) to the callback.
static void dispatch_frame(const uint8_t *frame, size_t len)
{
    if (s_central.rx_cb != NULL) {
        s_central.rx_cb(frame, len, s_central.rx_arg);
    } else {
        ESP_LOGW(TAG, "dropping frame (%u bytes): no rx callback", (unsigned)len);
    }
}

// Accumulate RX bytes and extract length-prefixed frames (2-byte BE length).
static void rx_ingest(const uint8_t *data, size_t len)
{
    size_t i = 0;
    while (i < len) {
        size_t need;
        size_t frame_len;

        if (s_central.rx_len >= 2) {
            frame_len = ((size_t)s_central.rx_buf[0] << 8) | s_central.rx_buf[1];
            if (frame_len + 2 > sizeof(s_central.rx_buf)) {
                ESP_LOGW(TAG, "rx frame too large (%u > %u); dropping buffered frame",
                         (unsigned)frame_len, (unsigned)(sizeof(s_central.rx_buf) - 2));
                s_central.rx_len = 0;
                return;
            }
            need = frame_len + 2 - s_central.rx_len;
        } else {
            need = 2 - s_central.rx_len;
        }
        if (need > len - i) {
            need = len - i;
        }
        memcpy(&s_central.rx_buf[s_central.rx_len], &data[i], need);
        s_central.rx_len += need;
        i += need;

        if (s_central.rx_len >= 2) {
            frame_len = ((size_t)s_central.rx_buf[0] << 8) | s_central.rx_buf[1];
            if (frame_len + 2 <= s_central.rx_len) {
                ESP_LOGI(TAG, "Tesla RX reassembled frame len=%u",
                         (unsigned)frame_len);
                dispatch_frame(&s_central.rx_buf[2], frame_len);
                memmove(s_central.rx_buf, &s_central.rx_buf[2 + frame_len],
                        s_central.rx_len - (2 + frame_len));
                s_central.rx_len -= (2 + frame_len);
            }
        }
    }
}

static void central_fail_cleanup(void);
static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg);
static int rx_dsc_disc_cb(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          uint16_t chr_val_handle,
                          const struct ble_gatt_dsc *dsc, void *arg);
static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg);

static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    int rc;
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale MTU callback handle=%u", conn_handle);
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGW(TAG, "MTU exchange failed: %d", error->status);
    } else {
        ESP_LOGI(TAG, "Central MTU: %u", mtu);
    }
    // NimBLE serializes GATT procedures (one at a time), so start service
    // discovery only after the MTU exchange completes.
    rc = ble_gattc_disc_svc_by_uuid(conn_handle, &s_svc_uuid.u, svc_disc_cb, arg);
    if (rc != 0) {
        ESP_LOGE(TAG, "service discovery start failed: %d", rc);
        central_done(conn_handle, arg, false);
    }
    return 0;
}

// Subscribe to both vehicle-command response modes; the working pairing path
// receives terminal VCSEC responses as either notifications or indications.
// The CCCD is discovered explicitly instead of assuming it follows the value
// handle. READY is reported only from cccd_write_cb after the peer responds.
static void subscribe_indicate(uint16_t conn_handle)
{
    const uint8_t cccd[2] = { 0x03, 0x00 };   // enable notifications + indications
    s_central.state = ST_SUBSCRIBING;
    ESP_LOGI(TAG, "writing Tesla CCCD handle=%u value=0x0003",
             s_central.rx_cccd_handle);
    int rc = ble_gattc_write_flat(conn_handle, s_central.rx_cccd_handle,
                                  cccd, sizeof(cccd), cccd_write_cb,
                                  (void *)(uintptr_t)s_central.generation);
    if (rc != 0) {
        ESP_LOGE(TAG, "cccd write start failed: handle=%u rc=%d",
                 s_central.rx_cccd_handle, rc);
        central_done(conn_handle, (void *)(uintptr_t)s_central.generation, false);
    }
}

static int tx_write_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale Tesla TX completion handle=%u", conn_handle);
        return 0;
    }
    s_central.tx_status = error->status;
    ESP_LOGD(TAG, "Tesla TX chunk complete handle=%u status=%u",
             conn_handle, (unsigned)error->status);
    if (s_central.tx_done_sem != NULL) {
        xSemaphoreGive(s_central.tx_done_sem);
    }
    return 0;
}

static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale Tesla CCCD completion handle=%u", conn_handle);
        return 0;
    }
    ESP_LOGI(TAG, "Tesla CCCD write complete handle=%u status=%u",
             s_central.rx_cccd_handle, (unsigned)error->status);
    central_done(conn_handle, arg, error->status == 0);
    return 0;
}

static int rx_dsc_disc_cb(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          uint16_t chr_val_handle,
                          const struct ble_gatt_dsc *dsc, void *arg)
{
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale Tesla RX descriptor callback handle=%u",
                 conn_handle);
        return 0;
    }
    if (error->status == 0 && dsc != NULL) {
        ESP_LOGI(TAG, "Tesla RX descriptor handle=%u", dsc->handle);
        if (ble_uuid_cmp(&dsc->uuid.u,
                         BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) == 0) {
            s_central.rx_cccd_handle = dsc->handle;
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (s_central.rx_cccd_handle == 0) {
            ESP_LOGE(TAG, "Tesla RX CCCD (0x2902) not found for value handle=%u",
                     chr_val_handle);
            central_done(conn_handle, arg, false);
        } else {
            subscribe_indicate(conn_handle);
        }
    } else {
        ESP_LOGE(TAG, "Tesla RX descriptor discovery failed: status=%u",
                 (unsigned)error->status);
        central_done(conn_handle, arg, false);
    }
    return 0;
}

static int chr_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale Tesla characteristic callback handle=%u",
                 conn_handle);
        return 0;
    }
    if (error->status == 0 && chr != NULL) {
        if (ble_uuid_cmp(&chr->uuid.u, &s_write_uuid.u) == 0) {
            s_central.tx_handle = chr->val_handle;
            s_central.tx_properties = chr->properties;
            ESP_LOGI(TAG, "found write characteristic (handle %u properties=0x%02x)",
                     chr->val_handle, chr->properties);
        } else if (ble_uuid_cmp(&chr->uuid.u, &s_indicate_uuid.u) == 0) {
            s_central.rx_handle = chr->val_handle;
            s_central.rx_def_handle = chr->def_handle;
            ESP_LOGI(TAG, "found indicate characteristic (handle %u)", chr->val_handle);
        }
        // Once the next characteristic definition is known, descriptors for
        // the Tesla RX characteristic end immediately before it. This avoids
        // accidentally selecting a CCCD belonging to another characteristic.
        if (s_central.rx_def_handle != 0 &&
            chr->def_handle > s_central.rx_def_handle &&
            s_central.rx_dsc_end == 0) {
            s_central.rx_dsc_end = (uint16_t)(chr->def_handle - 1);
        }
        return 0;
    }
    // Nonzero status (or the terminal status-0 chr==NULL call) ends discovery.
    if (s_central.tx_handle != 0 && s_central.rx_handle != 0) {
        uint16_t dsc_end = s_central.rx_dsc_end != 0 ?
                               s_central.rx_dsc_end : s_central.svc_end;
        int rc = ble_gattc_disc_all_dscs(conn_handle, s_central.rx_handle,
                                         dsc_end, rx_dsc_disc_cb, arg);
        if (rc != 0) {
            ESP_LOGE(TAG, "Tesla RX descriptor discovery start failed: %d", rc);
            central_done(conn_handle, arg, false);
        }
    } else {
        ESP_LOGE(TAG, "missing vehicle characteristics (err=%u)",
                 (unsigned)error->status);
        central_done(conn_handle, arg, false);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (!connection_current(conn_handle, arg)) {
        ESP_LOGW(TAG, "ignoring stale Tesla service callback handle=%u",
                 conn_handle);
        return 0;
    }
    if (error->status == 0 && svc != NULL) {
        if (ble_uuid_cmp(&svc->uuid.u, &s_svc_uuid.u) == 0) {
            s_central.svc_start = svc->start_handle;
            s_central.svc_end = svc->end_handle;
        }
        return 0;
    }
    // Nonzero status (or terminal status-0 svc==NULL) ends service discovery.
    if (s_central.svc_start != 0) {
        int rc = ble_gattc_disc_all_chrs(conn_handle, s_central.svc_start,
                                         s_central.svc_end, chr_disc_cb, arg);
        if (rc != 0) {
            ESP_LOGE(TAG, "chr discovery start failed: %d", rc);
            central_done(conn_handle, arg, false);
        }
    } else {
        ESP_LOGE(TAG, "vehicle service not found (err=%u)",
                 (unsigned)error->status);
        central_done(conn_handle, arg, false);
    }
    return 0;
}

static int central_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
        if (!generation_current(arg)) {
            ESP_LOGW(TAG, "ignoring stale central connect event generation=%lu",
                     (unsigned long)(uintptr_t)arg);
            if (event->connect.status == 0) {
                (void)ble_gap_terminate(event->connect.conn_handle,
                                        BLE_ERR_REM_USER_CONN_TERM);
            }
            break;
        }
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "central connect failed: %d", event->connect.status);
            central_done_connect(arg, false);
            s_central.state = ST_IDLE;
            break;
        }
        // A straggler from a timed-out/cancelled connect: drop it before
        // discovery so it can't wedge the state machine or leak a BLE slot.
        if (s_central.state != ST_CONNECTING) {
            ESP_LOGW(TAG, "dropping late central connection (timed out/cancelled)");
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }
        s_central.conn_handle = event->connect.conn_handle;
        s_central.state = ST_DISCOVERING;
        ESP_LOGI(TAG, "central connected (handle=%u)", s_central.conn_handle);
        // GATT procedures run serially; mtu_cb then starts service discovery.
        int rc = ble_gattc_exchange_mtu(s_central.conn_handle, mtu_cb, arg);
        if (rc != 0) {
            ESP_LOGE(TAG, "MTU exchange start failed: %d", rc);
            central_done(s_central.conn_handle, arg, false);
        }
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        if (!generation_current(arg) ||
            event->disconnect.conn.conn_handle != s_central.conn_handle) {
            ESP_LOGW(TAG, "ignoring stale central disconnect handle=%u generation=%lu",
                     event->disconnect.conn.conn_handle,
                     (unsigned long)(uintptr_t)arg);
            break;
        }
        ESP_LOGI(TAG, "central disconnected (handle=%u, reason=0x%x, state=%d)",
                 event->disconnect.conn.conn_handle, event->disconnect.reason,
                 (int)s_central.state);
        central_fail_cleanup();
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (connection_current(event->notify_rx.conn_handle, arg) &&
            event->notify_rx.om != NULL) {
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            ESP_LOGI(TAG, "Tesla RX %s attr=%u raw_len=%u",
                     event->notify_rx.indication ? "indication" : "notification",
                     event->notify_rx.attr_handle, len);
            if (len <= sizeof(s_notify_buf)) {
                int rc = ble_hs_mbuf_to_flat(event->notify_rx.om, s_notify_buf,
                                              sizeof(s_notify_buf), &len);
                if (rc != 0) {
                    ESP_LOGW(TAG, "Tesla RX mbuf flatten failed rc=%d", rc);
                } else {
                    rx_ingest(s_notify_buf, len);
                }
            } else {
                ESP_LOGW(TAG, "rx chunk too large (%u)", len);
            }
        }
        break;

    default:
        break;
    }
    return 0;
}

static esp_err_t central_connect_start(const void *addr)
{
    struct ble_gap_conn_params params;
    const ble_addr_t *peer = (const ble_addr_t *)addr;
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    int rc;

    if (!ble_hs_synced()) {
        return ESP_ERR_INVALID_STATE;
    }
    ble_hs_id_infer_auto(0, &own_addr_type);

    memset(&params, 0, sizeof(params));
    params.scan_itvl = 0x10;
    params.scan_window = 0x10;
    params.itvl_min = 96;        // ~120 ms connection interval
    params.itvl_max = 160;       // ~200 ms
    params.latency = 0;
    // 20 s supervision timeout: a silent-but-awake car in the enrollment tap
    // window used to drop the link (0x208) within the 60 s window; the pairing
    // task's GATT-read keepalive (tesla_ble_keepalive) keeps a live link fresh.
    params.supervision_timeout = 2000;
    params.min_ce_len = 0;
    params.max_ce_len = 0;

    s_central.state = ST_CONNECTING;
    rc = ble_gap_connect(own_addr_type, peer, TESLA_CONNECT_TIMEOUT_MS,
                         &params, central_gap_event_handler,
                         (void *)(uintptr_t)s_central.generation);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Reconcile asynchronous transport state during a disconnect. The binary GATT
// semaphore is released only for an owner that actually acquired it. A TX
// owner is given one completion wake-up so its task can observe tx_abort and
// release the semaphore itself; it is never released here on its behalf.
static void reconcile_transport_after_abort(void)
{
    if (s_central.gatt_owner == GATT_OWNER_KEEPALIVE) {
        s_central.keepalive_pending = false;
        s_central.gatt_owner = GATT_OWNER_NONE;
        if (s_central.gatt_sem != NULL) {
            xSemaphoreGive(s_central.gatt_sem);
        }
    } else if (s_central.gatt_owner == GATT_OWNER_TX) {
        s_central.tx_abort = true;
        s_central.tx_status = BLE_HS_ENOTCONN;
        while (s_central.tx_done_sem != NULL &&
               xSemaphoreTake(s_central.tx_done_sem, 0) == pdTRUE) {
        }
        // Wake a TX task blocked on the current write completion. It checks
        // tx_abort before accepting this as a real completion.
        if (s_central.tx_done_sem != NULL) {
            xSemaphoreGive(s_central.tx_done_sem);
        }
    } else {
        s_central.keepalive_pending = false;
        s_central.tx_status = BLE_HS_EUNKNOWN;
        while (s_central.tx_done_sem != NULL &&
               xSemaphoreTake(s_central.tx_done_sem, 0) == pdTRUE) {
        }
    }
    if (s_central.gatt_owner == GATT_OWNER_NONE) {
        s_central.tx_status = BLE_HS_EUNKNOWN;
        while (s_central.tx_done_sem != NULL &&
               xSemaphoreTake(s_central.tx_done_sem, 0) == pdTRUE) {
        }
    }
}

// GAP termination is asynchronous. Keep the old generation alive while
// waiting for its disconnect event, but block a new attempt from reusing the
// handle until that event arrives. If the controller does not report it in
// time, advance the generation so every late callback is harmless.
static void central_abort_and_wait(uint32_t timeout_ms)
{
    if (s_central.state == ST_IDLE) {
        return;
    }
    if (s_central.conn_handle != 0) {
        uint16_t handle = s_central.conn_handle;
        s_central.state = ST_DISCONNECTING;
        int rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
        ESP_LOGI(TAG, "central terminate handle=%u rc=%d", handle, rc);
        if (rc == 0 && s_central.disconnect_sem != NULL &&
            xSemaphoreTake(s_central.disconnect_sem,
                           pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            return;
        }
        ESP_LOGW(TAG, "central disconnect completion timed out handle=%u", handle);
    } else {
        int rc = ble_gap_conn_cancel();
        ESP_LOGI(TAG, "central connect cancel rc=%d", rc);
    }
    reconcile_transport_after_abort();
    s_central.generation++;
    if (s_central.generation == 0) {
        s_central.generation = 1;
    }
    s_central.state = ST_IDLE;
    s_central.conn_handle = 0;
}

esp_err_t tesla_ble_connect(const void *addr, uint32_t timeout_ms)
{
    esp_err_t err;

    if (addr == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_central.state == ST_DISCONNECTING) {
        central_abort_and_wait(2000);
    }
    if (s_central.state != ST_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_central.ready_sem == NULL) {
        s_central.ready_sem = xSemaphoreCreateBinary();
        if (s_central.ready_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_central.tx_done_sem == NULL) {
        s_central.tx_done_sem = xSemaphoreCreateBinary();
        if (s_central.tx_done_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_central.disconnect_sem == NULL) {
        s_central.disconnect_sem = xSemaphoreCreateBinary();
        if (s_central.disconnect_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_central.gatt_sem == NULL) {
        s_central.gatt_sem = xSemaphoreCreateBinary();
        if (s_central.gatt_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
        xSemaphoreGive(s_central.gatt_sem);
    }
    while (xSemaphoreTake(s_central.ready_sem, 0) == pdTRUE) {
    }
    while (xSemaphoreTake(s_central.disconnect_sem, 0) == pdTRUE) {
    }
    s_central.generation++;
    if (s_central.generation == 0) {
        s_central.generation = 1;
    }
    s_central.conn_handle = 0;
    s_central.svc_start = s_central.svc_end = 0;
    s_central.tx_handle = s_central.rx_handle = 0;
    s_central.rx_def_handle = s_central.rx_dsc_end = 0;
    s_central.rx_cccd_handle = 0;
    s_central.tx_properties = 0;
    s_central.rx_len = 0;
    s_central.connect_ok = false;

    if (central_connect_start(addr) != ESP_OK) {
        s_central.state = ST_IDLE;
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_central.ready_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "connect timed out");
        central_abort_and_wait(2000);
        return ESP_ERR_TIMEOUT;
    }
    err = s_central.connect_ok ? ESP_OK : ESP_FAIL;
    if (err == ESP_OK) {
        s_central.state = ST_READY;
    } else {
        central_abort_and_wait(2000);
    }
    return err;
}

esp_err_t tesla_ble_send(const uint8_t *data, size_t len)
{
    uint8_t framed[900];
    size_t total = len + 2;
    uint16_t mtu;
    size_t off = 0;

    if (s_central.state != ST_READY || s_central.conn_handle == 0 ||
        s_central.tx_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (total > sizeof(framed)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_central.gatt_sem, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Tesla TX rejected: another GATT procedure is active");
        return ESP_ERR_INVALID_STATE;
    }
    s_central.gatt_owner = GATT_OWNER_TX;
    s_central.tx_abort = false;
    s_central.tx_status = BLE_HS_EUNKNOWN;
    while (xSemaphoreTake(s_central.tx_done_sem, 0) == pdTRUE) {
    }
    framed[0] = (uint8_t)(len >> 8);
    framed[1] = (uint8_t)(len & 0xFF);
    memcpy(&framed[2], data, len);

    uint16_t conn_handle = s_central.conn_handle;
    mtu = ble_att_mtu(conn_handle);
    size_t chunk = mtu > 3 ? (size_t)(mtu - 3) : 20;
    if (chunk > UINT16_MAX) {
        chunk = UINT16_MAX;
    }
    ESP_LOGI(TAG, "Tesla TX frame len=%u mtu=%u chunk=%u mode=write-response",
             (unsigned)len, mtu, (unsigned)chunk);

    while (off < total) {
        uint16_t n = (uint16_t)((total - off < chunk) ?
                                    (total - off) : chunk);
        while (xSemaphoreTake(s_central.tx_done_sem, 0) == pdTRUE) {
        }
        if (s_central.tx_abort) {
            ESP_LOGW(TAG, "Tesla TX aborted before chunk offset=%u", (unsigned)off);
            s_central.gatt_owner = GATT_OWNER_NONE;
            xSemaphoreGive(s_central.gatt_sem);
            return ESP_ERR_INVALID_STATE;
        }
        s_central.tx_status = BLE_HS_EUNKNOWN;
        int rc = ble_gattc_write_flat(conn_handle, s_central.tx_handle,
                                      &framed[off], n, tx_write_cb,
                                      (void *)(uintptr_t)s_central.generation);
        ESP_LOGD(TAG, "Tesla TX chunk start offset=%u len=%u rc=%d",
                 (unsigned)off, n, rc);
        if (rc == 0 &&
            xSemaphoreTake(s_central.tx_done_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGE(TAG, "Tesla TX chunk completion timeout offset=%u",
                     (unsigned)off);
            s_central.gatt_owner = GATT_OWNER_NONE;
            xSemaphoreGive(s_central.gatt_sem);
            return ESP_ERR_TIMEOUT;
        }
        if (s_central.tx_abort) {
            ESP_LOGW(TAG, "Tesla TX aborted during chunk offset=%u", (unsigned)off);
            s_central.gatt_owner = GATT_OWNER_NONE;
            xSemaphoreGive(s_central.gatt_sem);
            return ESP_ERR_INVALID_STATE;
        }
        if (rc == 0) {
            rc = s_central.tx_status;
        }
        if (rc != 0) {
            ESP_LOGE(TAG, "Tesla TX chunk failed offset=%u len=%u rc=%d",
                     (unsigned)off, n, rc);
            xSemaphoreGive(s_central.gatt_sem);
            s_central.gatt_owner = GATT_OWNER_NONE;
            return ESP_FAIL;
        }
        ESP_LOGD(TAG, "Tesla TX chunk offset=%u len=%u", (unsigned)off, n);
        off += n;
    }
    s_central.gatt_owner = GATT_OWNER_NONE;
    xSemaphoreGive(s_central.gatt_sem);
    return ESP_OK;
}

// Link-only keepalive: a plain ATT read of the RX characteristic's CCCD.
//
// Two approaches were rejected on-car:
//   - READING char 0x213 consumes the car's pending VCSEC response, so the
//     tap result was lost and enrollment only completed via the post-timeout
//     handshake fallback.
//   - WRITING GET_STATUS to 0x212 consumed nothing, but the car went silent
//     once those writes began mid-window: no terminal response ever arrived.
// A descriptor read resets the supervision timer without touching the VCSEC
// stack or pending notifications; the tap response reaches rx_ingest intact.
//
// This callback and BLE_GAP_EVENT_NOTIFY_RX both run on the NimBLE host task,
// so sharing s_notify_buf and the rx framing state is safe.
static int keepalive_read_cb(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    if (connection_current(conn_handle, arg) &&
        s_central.keepalive_pending &&
        s_central.gatt_owner == GATT_OWNER_KEEPALIVE) {
        s_central.keepalive_pending = false;
        s_central.gatt_owner = GATT_OWNER_NONE;
        xSemaphoreGive(s_central.gatt_sem);
    }
    ESP_LOGI(TAG, "Tesla link keepalive complete status=%u",
             (unsigned)error->status);
    return 0;
}
// Link-only keepalive for quiet waits (e.g. the NFC-card tap window).
// No-op when not connected.
esp_err_t tesla_ble_keepalive(void)
{
    if (s_central.state != ST_READY || s_central.conn_handle == 0 ||
        s_central.rx_cccd_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_central.gatt_sem, 0) != pdTRUE) {
        ESP_LOGD(TAG, "Tesla keepalive skipped: GATT procedure busy");
        return ESP_ERR_INVALID_STATE;
    }
    s_central.gatt_owner = GATT_OWNER_KEEPALIVE;
    s_central.keepalive_pending = true;

    uint16_t conn_handle = s_central.conn_handle;
    int rc = ble_gattc_read(conn_handle, s_central.rx_cccd_handle,
                            keepalive_read_cb,
                            (void *)(uintptr_t)s_central.generation);
    ESP_LOGD(TAG, "Tesla keepalive read start rc=%d", rc);
    if (rc != 0) {
        s_central.keepalive_pending = false;
        s_central.gatt_owner = GATT_OWNER_NONE;
        xSemaphoreGive(s_central.gatt_sem);
    }
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

void tesla_ble_disconnect(void)
{
    central_abort_and_wait(2000);
}

void tesla_ble_set_rx_cb(tesla_ble_rx_fn_t cb, void *arg)
{
    s_central.rx_cb = cb;
    s_central.rx_arg = arg;
}

static void central_fail_cleanup(void)
{
    central_state_t prev = s_central.state;
    reconcile_transport_after_abort();
    s_central.state = ST_IDLE;
    s_central.conn_handle = 0;
    // A peer drop during connect/discovery fails a blocked
    // tesla_ble_connect() fast, instead of riding out the full timeout with
    // a misleading TIMEOUT result.
    if (prev == ST_CONNECTING || prev == ST_DISCOVERING ||
        prev == ST_SUBSCRIBING) {
        s_central.connect_ok = false;
        if (s_central.ready_sem != NULL) {
            xSemaphoreGive(s_central.ready_sem);
        }
    }
    if (s_central.disconnect_sem != NULL) {
        xSemaphoreGive(s_central.disconnect_sem);
    }
}
