/*
 * Present-key enrollment: generate a P-256 keypair, send a present-key
 * addKeyToWhitelistAndAddPermissions request over the central BLE link, and
 * keep reading VCSEC responses until the whitelist operation completes or the
 * tap window times out, then persist the key. Reuses the tesla_ble_adapter
 * central transport (connect, length-framed send, frame RX callback).
 * (The message is sent unsigned — see tesla_pairing.h for why that's safe.)
 */

#include "tesla_pairing.h"

#include "tesla_ble_adapter.h"
#include "ble_appchan.h"
#include "led.h"
#include "protobuf_build.h"
#include "session.h"
#include "universal_message.pb.h"
#include "vcsec.pb.h"

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "tesla_pairing";

// Enrolled role: read + charge only, presented as an Android-style device.
#define ENROLL_ROLE        Keys_Role_ROLE_CHARGING_MANAGER
#define ENROLL_FORM_FACTOR VCSEC_KeyFormFactor_KEY_FORM_FACTOR_ANDROID_DEVICE

// Timing: how long to wait for the owner's card tap + UI confirm, how long
// between enrollment attempts, and how many attempts before going idle.
#define TAP_TIMEOUT_MS      60000
#define RESPONSE_TIMEOUT_MS 3000
#define CONNECT_TIMEOUT_MS  20000
#define RETRY_DELAY_S       30
#define MAX_ATTEMPTS        3
#define CONFIG_WAIT_MS      1000
// How often the task re-checks the app 'start' latch once a car is staged.
#define STAGE_POLL_MS       200

typedef struct {
    uint16_t len;
    uint8_t  data[TESLA_RX_FRAME_MAX];
} pairing_frame_t;

static QueueHandle_t s_rxq;

// Keep the shared 600-byte frame bound out of the NimBLE host and pairing
// task stacks. The callback and task use separate queue-item scratch objects;
// the single work buffer is reused only between the sequential send/receive
// phases of an enrollment or verification attempt.
static pairing_frame_t s_pairing_cb_frame;
static pairing_frame_t s_pairing_task_frame;
static uint8_t s_pairing_work[TESLA_RX_FRAME_MAX];

// App-triggered 'go' latch: enrollment runs only while this is set. Written by
// the app-channel host task (start/cancel/reset), read/cleared by the pairing
// task; volatile is enough for a one-word bool.
static volatile bool s_app_allowed = false;

// Hardware-RNG wrapper for keypair generation.
static int hw_rng(void *ctx, uint8_t *buf, size_t len)
{
    (void)ctx;
    esp_fill_random(buf, len);
    return 0;
}

static void pairing_rx_cb(const uint8_t *data, size_t len, void *arg)
{
    (void)arg;
    if (s_rxq == NULL || len == 0) {
        ESP_LOGW(TAG, "pairing RX drop: queue unavailable or empty frame len=%u",
                 (unsigned)len);
        return;
    }
    if (len > TESLA_RX_FRAME_MAX) {
        ESP_LOGW(TAG, "pairing RX drop: frame too large len=%u max=%u",
                 (unsigned)len, (unsigned)TESLA_RX_FRAME_MAX);
        return;
    }
    ESP_LOGI(TAG, "pairing RX frame len=%u", (unsigned)len);
    s_pairing_cb_frame.len = (uint16_t)len;
    memcpy(s_pairing_cb_frame.data, data, len);
    if (xQueueSend(s_rxq, &s_pairing_cb_frame, 0) != pdTRUE) {
        ESP_LOGW(TAG, "pairing RX drop: queue full depth=%u",
                 (unsigned)uxQueueMessagesWaiting(s_rxq));
    } else {
        ESP_LOGD(TAG, "pairing RX queued depth=%u",
                 (unsigned)uxQueueMessagesWaiting(s_rxq));
    }
}

static esp_err_t pairing_recv(uint8_t *buf, size_t cap, size_t *out_len,
                              uint32_t timeout_ms)
{
    if (xQueueReceive(s_rxq, &s_pairing_task_frame,
                      (TickType_t)pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_pairing_task_frame.len > cap) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(buf, s_pairing_task_frame.data, s_pairing_task_frame.len);
    if (out_len != NULL) {
        *out_len = s_pairing_task_frame.len;
    }
    return ESP_OK;
}

// Classify a response frame. Returns 1 on terminal success (the whitelist
// operation completed, or the key is already on the whitelist), -1 on terminal
// failure (a whitelist-operation error code or a protocol-layer fault), or 0 to
// keep waiting (OPERATIONSTATUS_WAIT / empty / non-status).
static int pairing_ingest(const uint8_t *frame, size_t len, uint32_t *info_out)
{
    UniversalMessage_RoutableMessage rm;
    VCSEC_FromVCSECMessage from;
    const uint8_t *payload = frame;
    size_t plen = len;

    if (info_out != NULL) {
        *info_out = UINT32_MAX;
    }

    // nanopb only writes a which_* selector when that member is on the wire;
    // zero the structs so a WAIT reply (commandStatus with no sub_status) is
    // read as "keep waiting" rather than as uninitialized stack garbage.
    memset(&rm, 0, sizeof(rm));
    memset(&from, 0, sizeof(from));

    // The car replies with a RoutableMessage whose protobuf_message_as_bytes
    // holds the VCSEC.FromVCSECMessage (plaintext — no session, no encryption).
    int routable_rc = tesla_pb_decode_routable(frame, len, &rm);
    if (routable_rc == 0) {
        ESP_LOGI(TAG, "pairing RoutableMessage decode=ok payload=%u",
                 (unsigned)rm.which_payload);
    } else {
        ESP_LOGW(TAG, "pairing RoutableMessage decode=fail len=%u",
                 (unsigned)len);
    }

    if (routable_rc == 0 &&
        rm.which_payload == (pb_size_t)UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag) {
        payload = rm.payload.protobuf_message_as_bytes.bytes;
        plen = rm.payload.protobuf_message_as_bytes.size;
        ESP_LOGI(TAG, "pairing RoutableMessage payload=protobuf len=%u",
                 (unsigned)plen);
    } else if (rm.has_signedMessageStatus &&
               rm.signedMessageStatus.signed_message_fault !=
                   UniversalMessage_MessageFault_E_MESSAGEFAULT_ERROR_NONE) {
        // Protocol-layer rejection (e.g. INVALID_SIGNATURE): nothing to retry
        // around — surface it for diagnosis and stop.
        ESP_LOGW(TAG, "enrollment rejected at protocol layer (signed_message_fault=%d)",
                 (int)rm.signedMessageStatus.signed_message_fault);
        return -1;
    } else if (routable_rc == 0) {
        ESP_LOGI(TAG, "pairing RoutableMessage payload=non-protobuf selector=%u",
                 (unsigned)rm.which_payload);
    }

    int vcsec_rc = tesla_pb_decode_vcsec_from(payload, plen, &from);
    if (vcsec_rc != 0) {
        ESP_LOGW(TAG, "pairing FromVCSEC decode=fail len=%u", (unsigned)plen);
        // Dump first 48 bytes for diagnosis — helps identify whether the frame
        // is a RoutableMessage the car wrapped differently, or an unknown type.
        if (plen >= 2) {
            char hex[97];
            size_t n = plen < 48 ? plen : 48;
            for (size_t i = 0; i < n; i++) {
                snprintf(hex + i * 2, 3, "%02x", payload[i]);
            }
            hex[n * 2] = '\0';
            ESP_LOGW(TAG, "pairing frame hex: %s", hex);
        }
        return 0;
    }
    ESP_LOGI(TAG, "pairing FromVCSEC decode=ok submessage=%u",
             (unsigned)from.which_sub_message);
    uint32_t whitelist_info = UINT32_MAX;
    tesla_whitelist_phase_t phase = tesla_vcsec_whitelist_ingest(&from,
                                                                  &whitelist_info);
    if (info_out != NULL && whitelist_info != UINT32_MAX) {
        *info_out = whitelist_info;
    }
    ESP_LOGI(TAG, "pairing whitelist classifier phase=%u info=%lu",
             (unsigned)phase, (unsigned long)whitelist_info);
    if (phase == TESLA_WHITELIST_SUCCESS) {
        ESP_LOGI(TAG, "pairing whitelist result=success");
        return 1;
    }
    if (phase == TESLA_WHITELIST_ERROR) {
        ESP_LOGW(TAG, "pairing whitelist result=error info=%lu",
                 (unsigned long)whitelist_info);
        return -1;
    }
    return 0;
}

// Enroll `key` (already generated) on the car at `addr`. Returns ESP_OK only
// after the car confirms the key is enrolled. On success the keypair, VIN, and
// car address are persisted. Persists VIN + address first and the key last, so
// the key blob is the "enrollment complete" flag (a half-written key can't
// wedge the client into a key-without-config state).
static esp_err_t tesla_pairing_enroll(const tesla_keypair_t *key,
                               const char *vin, const tesla_car_addr_t *addr)
{
    if (key == NULL || vin == NULL || strlen(vin) != 17 || addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *msg = s_pairing_work;
    size_t msg_len = 0;
    if (tesla_pb_build_enrollment(key, ENROLL_ROLE, ENROLL_FORM_FACTOR,
                                  msg, TESLA_RX_FRAME_MAX, &msg_len) != 0) {
        ESP_LOGE(TAG, "failed to build enrollment message");
        return ESP_FAIL;
    }

    // Own the RX path for the duration of the attempt (the client poll loop is
    // idle while no key is enrolled).
    tesla_ble_set_rx_cb(pairing_rx_cb, NULL);

    ESP_LOGI(TAG, "connecting to %02X:%02X:%02X:%02X:%02X:%02X for enrollment",
             addr->val[5], addr->val[4], addr->val[3], addr->val[2],
             addr->val[1], addr->val[0]);
    if (tesla_ble_connect(addr, CONNECT_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGW(TAG, "connect failed");
        tesla_ble_disconnect();
        return ESP_ERR_TIMEOUT;
    }

    led_set_color(LED_COLOR_CYAN);
    ESP_LOGI(TAG, "enrollment request sent — tap an NFC card on the center "
                  "console, then confirm on the touchscreen");
    if (tesla_ble_send(msg, msg_len) != ESP_OK) {
        ESP_LOGW(TAG, "enrollment send failed");
        tesla_ble_disconnect();
        led_set_color(LED_COLOR_RED);
        return ESP_FAIL;
    }

    // Arm the tap window only now: reporting at task start would start the
    // app's countdown before the car actually armed its window.
    ble_appchan_report_status(TESLA_LINK_PAIRING_WINDOW, 0xFF, 0xFF, 0xFF, 0,
                              TESLA_FAULT_NONE);

    uint32_t start = xTaskGetTickCount();
    uint32_t last_ka = 0;
    uint32_t info = 0;
    esp_err_t res = ESP_ERR_TIMEOUT;
    while ((uint32_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(TAP_TIMEOUT_MS)) {
        // A cancel (0x03) clears s_app_allowed: stop the tap wait immediately.
        if (!s_app_allowed) {
            res = ESP_ERR_INVALID_STATE;
            break;
        }
        size_t flen = 0;
        if (pairing_recv(s_pairing_work, TESLA_RX_FRAME_MAX, &flen,
                         RESPONSE_TIMEOUT_MS) != ESP_OK) {
            // Link-only ATT touch every ~4 s: resets supervision so a
            // quiet-but-awake car can't drop mid-window (on-car: reason
            // 0x208), without sending VCSEC traffic that could disturb the
            // armed whitelist operation. See tesla_ble_keepalive().
            uint32_t now = xTaskGetTickCount();
            if (now - last_ka >= pdMS_TO_TICKS(4000)) {
                last_ka = now;
                ESP_LOGI(TAG, "tap window: no response in %us, sending keepalive read",
                         (unsigned)(now - start) / 1000);
                tesla_ble_keepalive();
            }
            continue;   // still waiting for the owner's tap
        }
        int r = pairing_ingest(s_pairing_work, flen, &info);
        if (r == 1) {
            res = ESP_OK;
            break;
        }
        if (r < 0) {
            ESP_LOGW(TAG, "enrollment rejected (whitelistOperationInformation=%lu)",
                     (unsigned long)info);
            res = ESP_ERR_INVALID_STATE;
            break;
        }
        // Car responded (WAIT / status) — re-report PAIRING_WINDOW with the
        // "car ready" flag (flags bit0) so the app can tell the user the car
        // is armed and ready for the keycard tap. ble_appchan_report_status
        // suppresses identical frames, so this only fires once per state change.
        ESP_LOGI(TAG, "car responded (WAIT/status) — signaling app: car ready for tap");
        ble_appchan_report_status(TESLA_LINK_PAIRING_WINDOW, 0xFF, 0xFF, 0xFF,
                                  0x01, TESLA_FAULT_NONE);
    }

    tesla_ble_disconnect();

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "enrollment did not complete (%s)",
                 res == ESP_ERR_TIMEOUT ? "timed out awaiting tap" : "rejected");
        led_set_color(LED_COLOR_RED);
        return res;
    }

    if (tesla_storage_save_vin(vin) != ESP_OK ||
        tesla_storage_save_car_addr(addr) != ESP_OK ||
        tesla_storage_save_key(key) != ESP_OK) {
        ESP_LOGE(TAG, "enrollment succeeded but persistence failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "key enrolled (CHARGING_MANAGER); persisted for reboot");
    led_set_color(LED_COLOR_GREEN);
    return ESP_OK;
}

// ---- pairing task + provisioning ----

static char s_vin[18];
static tesla_car_addr_t s_car_addr;
// Written by the app-channel host task (configure/reset) and read by the
// pairing task; volatile to prevent the compiler caching it across the loop.
static volatile bool s_configured;

esp_err_t tesla_pairing_configure(const char *vin, const tesla_car_addr_t *addr)
{
    if (vin == NULL || strlen(vin) != 17 || addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(s_vin, vin, 17);
    s_vin[17] = '\0';
    s_car_addr = *addr;
    s_configured = true;
    return ESP_OK;
}

// App-triggered-only start. Requires a staged car; refuses while a key is
// already enrolled. The pairing task picks s_app_allowed up on its next loop
// iteration. Reports the interim CONNECTING state immediately so the app
// leaves "staged" the moment the user taps Connect (the car only arms its
// tap window after the DashKit has connected + written the enrollment
// request, which can take a few seconds).
esp_err_t tesla_pairing_start(void)
{
    if (tesla_storage_has_key()) {
        ESP_LOGW(TAG, "start ignored: a key is already enrolled");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_configured) {
        ESP_LOGW(TAG, "start ignored: no car staged yet");
        return ESP_ERR_INVALID_STATE;
    }
    s_app_allowed = true;
    ble_appchan_report_status(TESLA_LINK_CONNECTING, 0xFF, 0xFF, 0xFF, 0,
                              TESLA_FAULT_NONE);
    ESP_LOGI(TAG, "app start: enrollment armed for VIN %s", s_vin);
    return ESP_OK;
}

// Cancel an in-progress enrollment back to staged. Tear down any central link
// so an open tap window dies immediately.
esp_err_t tesla_pairing_cancel(void)
{
    s_app_allowed = false;
    tesla_ble_disconnect();
    ESP_LOGI(TAG, "app cancel: enrollment stopped");
    return ESP_OK;
}

// Factory-reset Tesla state: erase the enrolled key and any staging, so the
// board returns to "never enrolled". The next car sighting re-stages (0x05) and
// waits for an app start again.
esp_err_t tesla_pairing_reset(void)
{
    esp_err_t err = tesla_storage_erase_all();
    s_app_allowed = false;
    s_configured = false;
    tesla_ble_disconnect();
    ESP_LOGW(TAG, "app reset: Tesla key erased (app re-provisions via 0x04)");
    (void)err;
    return ESP_OK;
}

// If the tap window expires without a terminal whitelist response, the owner
// may still have tapped + confirmed (the car accepted the key but the
// confirmation frame was lost / misrouted). Rather than guessed failure,
// reconnect and run the VCSEC handshake: a whitelisted session proves the key
// was added. On success the keypair, VIN, and address are persisted (key last,
// so the key blob remains the "enrollment complete" flag).
static esp_err_t tesla_pairing_verify_enrolled(const tesla_keypair_t *key,
                                               const char *vin,
                                               const tesla_car_addr_t *addr)
{
    if (key == NULL || vin == NULL || strlen(vin) != 17 || addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tesla_ble_set_rx_cb(pairing_rx_cb, NULL);
    // Drain stale frames (a late reply from the dead tap loop must not answer
    // the fresh handshake).
    while (s_rxq != NULL && xQueueReceive(s_rxq, &s_pairing_task_frame, 0) == pdTRUE) {
    }

    if (tesla_ble_connect(addr, CONNECT_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGW(TAG, "verify: connect failed");
        tesla_ble_disconnect();
        return ESP_ERR_TIMEOUT;
    }

    uint8_t routing[16], challenge[16], req[400];
    uint8_t *resp = s_pairing_work;
    size_t req_len = 0, resp_len = 0;
    esp_fill_random(routing, sizeof(routing));
    esp_fill_random(challenge, sizeof(challenge));
    if (tesla_build_handshake_request(TESLA_DOMAIN_VEHICLE_SECURITY, key->pub,
                                      routing, challenge,
                                      req, sizeof(req), &req_len) != 0) {
        ESP_LOGE(TAG, "verify: failed to build handshake request");
        tesla_ble_disconnect();
        return ESP_FAIL;
    }
    if (tesla_ble_send(req, req_len) != ESP_OK) {
        ESP_LOGW(TAG, "verify: handshake send failed");
        tesla_ble_disconnect();
        return ESP_FAIL;
    }

    // Wait for the handshake response matching OUR routing address; drop any
    // other frame that races in from the previous session.
    esp_err_t recv_err = ESP_ERR_TIMEOUT;
    uint32_t start = xTaskGetTickCount();
    while (recv_err != ESP_OK &&
           (uint32_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)) {
        if (xQueueReceive(s_rxq, &s_pairing_task_frame,
                          pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)) != pdTRUE) {
            break;
        }
        UniversalMessage_RoutableMessage m;
        memset(&m, 0, sizeof(m));
        if (tesla_pb_decode_routable(s_pairing_task_frame.data,
                                     s_pairing_task_frame.len, &m) != 0) {
            continue;
        }
        if (m.has_to_destination &&
            m.to_destination.which_sub_destination ==
                UniversalMessage_Destination_routing_address_tag &&
            m.to_destination.sub_destination.routing_address.size == 16 &&
            memcmp(m.to_destination.sub_destination.routing_address.bytes,
                   routing, 16) == 0) {
            resp_len = s_pairing_task_frame.len;
            memcpy(resp, s_pairing_task_frame.data, s_pairing_task_frame.len);
            recv_err = ESP_OK;
        }
        // Otherwise: not our response, keep waiting.
    }
    if (recv_err != ESP_OK) {
        ESP_LOGW(TAG, "verify: no handshake response (car asleep?)");
        tesla_ble_disconnect();
        return ESP_ERR_TIMEOUT;
    }

    tesla_session_t sess;
    tesla_session_init(&sess, TESLA_DOMAIN_VEHICLE_SECURITY, NULL);
    if (tesla_session_handshake(&sess, key, (const uint8_t *)vin, strlen(vin),
                                challenge, resp, resp_len,
                                hw_rng, NULL) != 0) {
        ESP_LOGW(TAG, "verify: handshake rejected");
        tesla_ble_disconnect();
        return ESP_ERR_INVALID_STATE;
    }
    tesla_ble_disconnect();
    if (!sess.whitelisted) {
        ESP_LOGW(TAG, "verify: handshake OK but key NOT on whitelist");
        return ESP_ERR_INVALID_STATE;
    }

    if (tesla_storage_save_vin(vin) != ESP_OK ||
        tesla_storage_save_car_addr(addr) != ESP_OK ||
        tesla_storage_save_key(key) != ESP_OK) {
        ESP_LOGE(TAG, "verify: enrollment confirmed but persistence failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "verify: key confirmed on whitelist; persisted for reboot");
    led_set_color(LED_COLOR_GREEN);
    return ESP_OK;
}

// Map an enrollment failure to the app-channel fault-detail byte.
static uint8_t fault_detail_for(esp_err_t e)
{
    if (e == ESP_ERR_TIMEOUT)      return TESLA_FAULT_TAP_TIMEOUT;
    if (e == ESP_ERR_INVALID_STATE) return TESLA_FAULT_REJECTED;
    if (e == ESP_FAIL)             return TESLA_FAULT_PERSIST;
    return TESLA_FAULT_PROTOCOL;
}

static void pairing_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (tesla_storage_has_key()) {
            // Already enrolled; the client poll loop owns the link and status
            // reporting. Idle until a re-pair / factory reset clears the key.
            vTaskDelay(pdMS_TO_TICKS(CONFIG_WAIT_MS));
            continue;
        }

        // No key yet: stage a known car (NVS resume or app provisioning) but
        // never begin on our own — the LEDs aren't a visible prompt.
        if (!s_configured) {
            if (tesla_storage_load_vin(s_vin, sizeof(s_vin)) == ESP_OK &&
                strlen(s_vin) == 17 &&
                tesla_storage_load_car_addr(&s_car_addr) == ESP_OK) {
                s_configured = true;   // staged (0x05), awaiting app start
            } else {
                // Nothing known: report "never enrolled" (0x00) and wait.
                ble_appchan_report_status(TESLA_LINK_NEVER_ENROLLED, 0xFF, 0xFF,
                                          0xFF, 0, TESLA_FAULT_NONE);
                vTaskDelay(pdMS_TO_TICKS(CONFIG_WAIT_MS));
                continue;
            }
        }

        if (!s_app_allowed) {
            // Staged (0x05): car known, waiting for the app to say go.
            ble_appchan_report_status(TESLA_LINK_STAGED, 0xFF, 0xFF, 0xFF, 0,
                                      TESLA_FAULT_NONE);
            vTaskDelay(pdMS_TO_TICKS(STAGE_POLL_MS));
            continue;
        }

        ESP_LOGI(TAG, "starting enrollment (VIN %s, role CHARGING_MANAGER)", s_vin);

        // Generate the keypair once and reuse it across attempts: if the owner
        // tapped + confirmed but the terminal response was lost, the retry
        // re-sends the SAME key and the car reports "already on whitelist",
        // which pairing_ingest treats as success (instead of orphaning the
        // enrolled key and adding a second one).
        tesla_keypair_t key;
        if (tesla_keypair_generate(&key, hw_rng, NULL) != 0) {
            ESP_LOGE(TAG, "keypair generation failed");
            s_app_allowed = false;
            ble_appchan_report_status(TESLA_LINK_ENROLLMENT_FAULT, 0xFF, 0xFF,
                                      0xFF, 0, TESLA_FAULT_PROTOCOL);
            continue;
        }

        bool enrolled = false;
        esp_err_t last_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < MAX_ATTEMPTS && !enrolled && s_app_allowed; attempt++) {
            esp_err_t e = tesla_pairing_enroll(&key, s_vin, &s_car_addr);
            // A cancel (0x03) clears s_app_allowed, which makes enroll abort
            // mid-window; stop immediately without retrying or surfacing a fault.
            if (e != ESP_OK && !s_app_allowed) {
                break;
            }
            last_err = e;
            if (e == ESP_OK) {
                ESP_LOGI(TAG, "enrollment complete; client poll loop takes over");
                enrolled = true;
                break;
            }

            // No terminal frame, but the key may still be enrolled (the
            // confirm frame was lost) — verify by handshake instead of
            // guessing failure. See tesla_pairing_verify_enrolled().
            if (e == ESP_ERR_TIMEOUT && s_app_allowed) {
                ESP_LOGW(TAG, "tap window expired; verifying enrollment with a handshake");
                if (tesla_pairing_verify_enrolled(&key, s_vin, &s_car_addr) == ESP_OK) {
                    ESP_LOGI(TAG, "enrollment confirmed via handshake verification");
                    enrolled = true;
                    break;
                }
                // Verification failed (car asleep/rejected): retreat to the
                // interim state while we back off and try again.
                ble_appchan_report_status(TESLA_LINK_CONNECTING, 0xFF, 0xFF, 0xFF,
                                          0, TESLA_FAULT_NONE);
            }
            ESP_LOGW(TAG, "enrollment attempt %d/%d failed (%s); retrying in %d s",
                     attempt + 1, MAX_ATTEMPTS, esp_err_to_name(e), RETRY_DELAY_S);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_S * 1000));
        }

        if (enrolled) {
            // Report success now so the app leaves the tap screen; the client
            // poll loop's own status arrives only after a full
            // connect→handshake→GET_STATUS cycle.
            ble_appchan_report_status(TESLA_LINK_ENROLLED_NOT_CONNECTED, 0xFF, 0xFF,
                                      0xFF, 0, TESLA_FAULT_NONE);
            // Client poll loop now owns the link + status. Clear the trigger so
            // a dropped key doesn't auto-renroll later.
            s_app_allowed = false;
            s_configured = false;
            continue;
        }

        if (!s_app_allowed) {
            // Canceled by the app: return to staged (0x05) and await a fresh
            // start — no fault is surfaced for a user-initiated cancel.
            continue;
        }

        // Gave up: surface a fault (0x04), then return to staged awaiting the
        // app again (keeping the staged VIN so a retry is one tap away).
        ESP_LOGW(TAG, "enrollment gave up after %d attempts; re-trigger via app",
                 MAX_ATTEMPTS);
        s_app_allowed = false;
        ble_appchan_report_status(TESLA_LINK_ENROLLMENT_FAULT, 0xFF, 0xFF, 0xFF,
                                  0, fault_detail_for(last_err));
    }
}

esp_err_t tesla_pairing_init(void)
{
    s_rxq = xQueueCreate(4, sizeof(pairing_frame_t));
    if (s_rxq == NULL) {
        ESP_LOGE(TAG, "failed to create rx queue");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(pairing_task, "tesla_pairing", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create pairing task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
