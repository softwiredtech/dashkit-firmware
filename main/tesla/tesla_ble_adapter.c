/*
 * Tesla BLE adapter — NimBLE central connection to the vehicle-command GATT
 * service.
 *
 * Phase 1 added observer scanning; Phase 2 adds the central path. This module
 * keeps the two strictly separate (ADR 0001 review note 2): the observer's
 * discovery handler and this central connect callback never feed the
 * peripheral server's slot table in main/ble/ble_server.c. Idle-disconnect
 * (connect → exchange → send → disconnect) is load-bearing per plan §6.
 */

#include "tesla_ble_adapter.h"
#include "tesla_advert_name.h"
#include "tesla_ble_storage.h"

#include "esp_log.h"
#include "esp_random.h"
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
#define TESLA_RX_BUF 600

// ---------------------------------------------------------------------------
// Shared central-connection state (declared before the observer so its helper
// functions can reference it).
// ---------------------------------------------------------------------------
typedef enum {
    ST_IDLE,
    ST_CONNECTING,
    ST_DISCOVERING,
    ST_SUBSCRIBING,
    ST_READY,
} central_state_t;

static struct {
    central_state_t state;
    uint16_t conn_handle;
    uint16_t svc_start, svc_end;
    uint16_t tx_handle, rx_handle;
    SemaphoreHandle_t ready_sem;   // binary: posted when READY or failed
    bool connect_ok;
    bool observer_running;
    // RX framing + dispatch
    uint8_t rx_buf[TESLA_RX_BUF];
    size_t  rx_len;
    tesla_ble_rx_fn_t rx_cb;
    void   *rx_arg;
} s_central;

// ---------------------------------------------------------------------------
// Observer scan (Phase 1) — independent of the central path.
// ---------------------------------------------------------------------------
static int discovery_event_handler(struct ble_gap_event *event, void *arg)
{
    static const char *fmt_str[] = { "none", "legacy", "modern" };
    const struct ble_gap_disc_desc *disc;
    struct ble_hs_adv_fields fields;
    enum tesla_name_format fmt;
    const uint8_t *mac;

    (void)arg;
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }
    disc = &event->disc;
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) {
        return 0;
    }
    if (fields.name == NULL || fields.name_len == 0) {
        return 0;
    }
    fmt = tesla_advert_name_format(fields.name, fields.name_len);
    mac = disc->addr.val;
    if (fmt != TESLA_NAME_NONE) {
        // The name/format is the feature's primary output, so keep it at INFO.
        // The full MAC is more identifying (a beacon is linkable across scans),
        // so gate it behind debug per the review privacy note.
        ESP_LOGI(TAG, "Tesla vehicle found: name=\"%.*s\" (format=%s), rssi=%d",
                 (int)fields.name_len, (const char *)fields.name, fmt_str[fmt],
                 (int)disc->rssi);
        ESP_LOGD(TAG, "  MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    } else {
        // Diagnostic (temporary INFO): surface every nearby advertisement name
        // so a live monitor shows what the observer actually sees. Intended to
        // prove/disprove the scan path; demote back to DEBUG once confirmed.
        ESP_LOGI(TAG, "advert seen: name=\"%.*s\" (format=-) rssi=%d, mac=%02X:%02X:%02X:%02X:%02X:%02X",
                 (int)fields.name_len, (const char *)fields.name, (int)disc->rssi,
                 mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    }
    // Persist every distinct advertisement name, flagging Tesla matches, so an
    // in-car run made with no live serial monitor is read off at the next boot
    // and we can see exactly what the car actually broadcasts.
    tesla_advert_log_add(fields.name, fields.name_len,
                         (uint8_t)(fmt != TESLA_NAME_NONE), (uint8_t)fmt,
                         mac, disc->rssi);
    return 0;
}

static esp_err_t start_scan(void)
{
    struct ble_gap_disc_params params;
    uint8_t own_addr_type;
    int rc;

    if (!ble_hs_synced()) {
        return ESP_ERR_INVALID_STATE;
    }
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }
    memset(&params, 0, sizeof(params));
    // Active scan: the local name may be delivered in ADV_IND or SCAN_RSP,
    // and passive scanning only ever sees the former (review finding). Active
    // scanning sends a SCAN_REQ and so surfaces either case. Harmless here —
    // DashKit already advertises as a peripheral, so the extra scan requests
    // don't conflict.
    params.passive = 0;
    params.filter_duplicates = 1;
    params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
    // Explicit scan cadence (0.625 ms units): ~30 ms interval, ~12.5 ms
    // window. Window < interval so the forever-scan doesn't run at full duty;
    // before, these were left 0 and silently picked up NimBLE's fast-scan
    // defaults.
    params.itvl = 0x30;      /* 0x30 * 0.625 ms = 30 ms scan interval */
    params.window = 0x14;    /* 0x14 * 0.625 ms = 12.5 ms active window */

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params,
                      discovery_event_handler, NULL);
    if (rc == BLE_HS_EALREADY) {
        return ESP_OK;
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "scan start failed: rc=%d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "scanning for Tesla advertisements (observer)");
    return ESP_OK;
}

static void scan_wait_task(void *arg)
{
    (void)arg;
    for (;;) {
        // Wait for a synced host. A host/controller reset drops sync and
        // re-enters this loop, so the observer re-arms itself after a reset
        // instead of silently never scanning again. ble_server.c's on_sync()
        // only restarts advertising and is peripheral-only, so the Tesla scan
        // can't lean on it (and this module must not touch ble_hs_cfg).
        while (!ble_hs_synced()) {
            s_central.observer_running = false;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (!s_central.observer_running) {
            if (start_scan() == ESP_OK) {
                s_central.observer_running = true;
            } else {
                ESP_LOGE(TAG, "Tesla observer failed to start scanning");
            }
        }
        // Park until sync is lost (host reset) or the central path has taken
        // over the controller. The central idle-disconnect re-arms the scan in
        // central_fail_cleanup(), so there is nothing to do here meanwhile.
        while (ble_hs_synced()) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

esp_err_t tesla_ble_adapter_observer_init(void)
{
    // Confirm power-ons: the boot counter advances every boot, so an unattended
    // run (e.g. in the car) shows up as a gap/advance in the sequence even if it
    // recorded no advertisements.
    ESP_LOGI(TAG, "boot #%u - dumping previous run's advertisement log",
             (unsigned)tesla_storage_boot_count());
    // Dump the previous run's advertisement names immediately, so a bench run
    // done without a serial monitor is read off at the next boot.
    tesla_advert_log_dump();
    if (xTaskCreate(scan_wait_task, "tesla_scan", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create scan task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

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
static void central_done(bool ok)
{
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
                ESP_LOGW(TAG, "rx frame too large (%u); resetting", (unsigned)frame_len);
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

static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    int rc;
    (void)arg;
    if (error->status != 0) {
        ESP_LOGW(TAG, "MTU exchange failed: %d", error->status);
    } else {
        ESP_LOGI(TAG, "Central MTU: %u", mtu);
    }
    // NimBLE serializes GATT procedures (one at a time), so start service
    // discovery only after the MTU exchange completes.
    rc = ble_gattc_disc_svc_by_uuid(conn_handle, &s_svc_uuid.u, svc_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "service discovery start failed: %d", rc);
        central_done(false);
    }
    return 0;
}

// Subscribe to the indicate characteristic (write 0x0002 to its CCCD).
static void subscribe_indicate(uint16_t conn_handle)
{
    const uint8_t cccd[2] = { 0x02, 0x00 };   // enable indications
    s_central.state = ST_SUBSCRIBING;
    int rc = ble_gattc_write_flat(conn_handle, s_central.rx_handle + 1,
                                  cccd, sizeof(cccd), NULL, NULL);
    if (rc == 0) {
        // The subscribe is fire-and-forget here (we optimistically mark ready;
        // an unsupported CCCD write would surface as a disconnect/timeout).
        central_done(true);
    } else {
        ESP_LOGE(TAG, "cccd write failed: %d", rc);
        central_done(false);
    }
}

static int chr_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (error->status == 0 && chr != NULL) {
        if (ble_uuid_cmp(&chr->uuid.u, &s_write_uuid.u) == 0) {
            s_central.tx_handle = chr->val_handle;
            ESP_LOGI(TAG, "found write characteristic (handle %u)", chr->val_handle);
        } else if (ble_uuid_cmp(&chr->uuid.u, &s_indicate_uuid.u) == 0) {
            s_central.rx_handle = chr->val_handle;
            ESP_LOGI(TAG, "found indicate characteristic (handle %u)", chr->val_handle);
        }
        return 0;
    }
    // Nonzero status (or the terminal status-0 chr==NULL call) ends discovery.
    if (s_central.tx_handle != 0 && s_central.rx_handle != 0) {
        subscribe_indicate(conn_handle);
    } else {
        ESP_LOGE(TAG, "missing vehicle characteristics (err=%u)",
                 (unsigned)error->status);
        central_done(false);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;
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
                                         s_central.svc_end, chr_disc_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "chr discovery start failed: %d", rc);
            central_done(false);
        }
    } else {
        ESP_LOGE(TAG, "vehicle service not found (err=%u)",
                 (unsigned)error->status);
        central_done(false);
    }
    return 0;
}

static int central_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "central connect failed: %d", event->connect.status);
            s_central.state = ST_IDLE;
            central_done(false);
            break;
        }
        s_central.conn_handle = event->connect.conn_handle;
        // A straggler from a timed-out/cancelled connect (review S3): the
        // waiter gave up or we cancelled, so drop it now before entering
        // discovery — otherwise it would wedge the state machine at
        // ST_DISCOVERING forever and leak a scarce BLE slot.
        if (s_central.state == ST_IDLE) {
            ESP_LOGW(TAG, "dropping late central connection (timed out/cancelled)");
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }
        s_central.state = ST_DISCOVERING;
        ESP_LOGI(TAG, "central connected (handle=%u)", s_central.conn_handle);
        // GATT procedures run serially; mtu_cb then starts service discovery.
        ble_gattc_exchange_mtu(s_central.conn_handle, mtu_cb, NULL);
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "central disconnected (handle=%u, reason=0x%x)",
                 event->disconnect.conn.conn_handle, event->disconnect.reason);
        central_fail_cleanup();
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == s_central.conn_handle &&
            event->notify_rx.om != NULL) {
            uint8_t buf[TESLA_RX_BUF];
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (len <= sizeof(buf)) {
                ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &len);
                rx_ingest(buf, len);
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
    // A passive observer scan and a connection attempt share the controller;
    // stop scanning before connecting.
    if (s_central.observer_running) {
        ble_gap_disc_cancel();
        s_central.observer_running = false;
    }
    ble_hs_id_infer_auto(0, &own_addr_type);

    memset(&params, 0, sizeof(params));
    params.scan_itvl = 0x10;
    params.scan_window = 0x10;
    params.itvl_min = 96;        // ~120 ms connection interval
    params.itvl_max = 160;       // ~200 ms
    params.latency = 0;
    params.supervision_timeout = 400;  // 4 s
    params.min_ce_len = 0;
    params.max_ce_len = 0;

    rc = ble_gap_connect(own_addr_type, peer, TESLA_CONNECT_TIMEOUT_MS,
                         &params, central_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        return ESP_FAIL;
    }
    s_central.state = ST_CONNECTING;
    return ESP_OK;
}

esp_err_t tesla_ble_connect(const void *addr, uint32_t timeout_ms)
{
    esp_err_t err;

    if (addr == NULL || s_central.state != ST_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_central.ready_sem == NULL) {
        s_central.ready_sem = xSemaphoreCreateBinary();
        if (s_central.ready_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_central.conn_handle = 0;
    s_central.svc_start = s_central.svc_end = 0;
    s_central.tx_handle = s_central.rx_handle = 0;
    s_central.rx_len = 0;
    s_central.connect_ok = false;

    if (central_connect_start(addr) != ESP_OK) {
        s_central.state = ST_IDLE;
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_central.ready_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "connect timed out");
        if (s_central.conn_handle != 0) {
            ble_gap_terminate(s_central.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        } else {
            // Connection request still pending at the controller: cancel it so
            // no late CONNECT event can arrive and wedge the state machine
            // (review S3).
            ble_gap_conn_cancel();
        }
        s_central.state = ST_IDLE;
        return ESP_ERR_TIMEOUT;
    }
    err = s_central.connect_ok ? ESP_OK : ESP_FAIL;
    if (err == ESP_OK) {
        s_central.state = ST_READY;
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
    framed[0] = (uint8_t)(len >> 8);
    framed[1] = (uint8_t)(len & 0xFF);
    memcpy(&framed[2], data, len);

    uint16_t conn_handle = s_central.conn_handle;
    mtu = ble_att_mtu(conn_handle);
    int chunk = (mtu > 0) ? (mtu - 3) : 20;   // 3-byte ATT header
    if (chunk < 1) chunk = 1;

    while (off < total) {
        uint16_t n = (uint16_t)((total - off < (size_t)chunk)
                                    ? (total - off) : (size_t)chunk);
        int rc = ble_gattc_write_flat(conn_handle, s_central.tx_handle,
                                      &framed[off], n, NULL, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "write failed at offset %u: %d", (unsigned)off, rc);
            return ESP_FAIL;
        }
        off += n;
    }
    return ESP_OK;
}

void tesla_ble_disconnect(void)
{
    if (s_central.conn_handle != 0 &&
        s_central.state != ST_IDLE) {
        ble_gap_terminate(s_central.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    central_fail_cleanup();
}

void tesla_ble_set_rx_cb(tesla_ble_rx_fn_t cb, void *arg)
{
    s_central.rx_cb = cb;
    s_central.rx_arg = arg;
}

static void central_fail_cleanup(void)
{
    s_central.state = ST_IDLE;
    s_central.conn_handle = 0;
    // Restart the observer scan after the car link is gone (idle-disconnect).
    if (ble_hs_synced() && !s_central.observer_running) {
        if (start_scan() == ESP_OK) {
            s_central.observer_running = true;
        }
    }
}
