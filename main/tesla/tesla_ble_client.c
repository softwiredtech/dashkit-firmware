/*
 * Tesla BLE client task — central vehicle-command session + GET_STATUS poll.
 */

#include "tesla_ble_client.h"
#include "tesla_ble_adapter.h"
#include "tesla_ble_storage.h"
#include "session.h"
#include "protobuf_build.h"
#include "vcsec.pb.h"
#include "universal_message.pb.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "tesla_client";

// Poll cadence + connection budget (plan §2/§6).
#define POLL_INTERVAL_S      10
#define POLLS_PER_CONN       5
#define RECONNECT_DELAY_S    30
#define RESPONSE_TIMEOUT_MS  3000
#define CONNECT_TIMEOUT_MS   20000
#define NO_KEY_DELAY_S       10

// Max BLE frame (framing + payload). 320 B is fine for Phase 2/3 VCSEC
// (GET_STATUS responses are small), but Phase 4 Infotainment responses
// (getVehicleData etc.) can exceed this — the reference transports
// up-to-1024-byte BLE messages. Grow RX_FRAME_MAX (and the transport's
// payload max) when the infotainment domain lands.
#define RX_FRAME_MAX 320
typedef struct {
    uint16_t len;
    uint8_t  data[RX_FRAME_MAX];
} rx_frame_t;

static QueueHandle_t s_rxq;

// Hardware-RNG wrapper for the session crypto (mbedTLS ECDH blinding + nonce).
static int hw_rng(void *ctx, uint8_t *buf, size_t len)
{
    (void)ctx;
    esp_fill_random(buf, len);
    return 0;
}

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static void client_rx_cb(const uint8_t *data, size_t len, void *arg)
{
    (void)arg;
    if (s_rxq == NULL || len == 0 || len > RX_FRAME_MAX) {
        return;
    }
    rx_frame_t f;
    f.len = (uint16_t)len;
    memcpy(f.data, data, len);
    if (xQueueSend(s_rxq, &f, 0) != pdTRUE) {
        ESP_LOGW(TAG, "rx queue full; dropping frame");
    }
}

// Wait for a frame whose to_destination.routing_address matches `routing`
// (the address we used for the outstanding request), so a late frame from a
// previous request never answers the current one.
static esp_err_t recv_for_route(uint8_t *buf, size_t cap, size_t *out_len,
                                uint32_t timeout_ms, const uint8_t routing[16])
{
    // Tick counter wraps (uint32): always subtract to compute what's left,
    // never compare absolute tick values directly (review N3).
    uint32_t start = xTaskGetTickCount();
    uint32_t total = pdMS_TO_TICKS(timeout_ms);

    while (true) {
        rx_frame_t f;
        uint32_t elapsed = (uint32_t)(xTaskGetTickCount() - start);
        uint32_t remain = (elapsed < total) ? total - elapsed : 0;

        if (xQueueReceive(s_rxq, &f, (TickType_t)remain) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        UniversalMessage_RoutableMessage m;
        if (tesla_pb_decode_routable(f.data, f.len, &m) != 0) {
            continue;   // unparseable frame: drop, keep waiting
        }
        if (m.has_to_destination &&
            m.to_destination.which_sub_destination ==
                UniversalMessage_Destination_routing_address_tag &&
            m.to_destination.sub_destination.routing_address.size == 16 &&
            memcmp(m.to_destination.sub_destination.routing_address.bytes,
                   routing, 16) == 0) {
            if (f.len > cap) {
                return ESP_ERR_INVALID_SIZE;
            }
            memcpy(buf, f.data, f.len);
            if (out_len != NULL) {
                *out_len = f.len;
            }
            return ESP_OK;
        }
        // Not our response; drop and keep waiting.
    }
}

// Returns the VIN as a local copy.
static bool load_config(tesla_keypair_t *key, tesla_car_addr_t *addr, char *vin)
{
    if (!tesla_storage_has_key()) {
        return false;
    }
    if (tesla_storage_load_key(key) != ESP_OK) {
        return false;
    }
    if (tesla_storage_load_car_addr(addr) != ESP_OK) {
        return false;
    }
    if (tesla_storage_load_vin(vin, 32) != ESP_OK || strlen(vin) != 17) {
        return false;
    }
    return true;
}

static const char *lock_name(int v)
{
    switch (v) {
    case VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_UNLOCKED: return "unlocked";
    case VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_LOCKED: return "locked";
    default: return "?";
    }
}

static const char *sleep_name(int v)
{
    switch (v) {
    case VCSEC_VehicleSleepStatus_E_VEHICLE_SLEEP_STATUS_AWAKE: return "awake";
    case VCSEC_VehicleSleepStatus_E_VEHICLE_SLEEP_STATUS_ASLEEP: return "asleep";
    default: return "unknown";
    }
}

static const char *presence_name(int v)
{
    switch (v) {
    case VCSEC_UserPresence_E_VEHICLE_USER_PRESENCE_PRESENT: return "present";
    case VCSEC_UserPresence_E_VEHICLE_USER_PRESENCE_NOT_PRESENT: return "absent";
    default: return "unknown";
    }
}

static void refresh_status(tesla_session_t *sess, const char *vin,
                           int *last_presence, int *last_lock, int *last_sleep)
{
    uint8_t payload[TESLA_PB_PAYLOAD_MAX], req[400];
    uint8_t request_hash[33], routing[16], uuid[16];
    size_t payload_len, req_len, req_hash_len;

    if (tesla_pb_encode_vcsec_status(payload, sizeof(payload), &payload_len) != 0 ||
        payload_len == 0) {
        ESP_LOGE(TAG, "failed to build GET_STATUS");
        return;
    }
    esp_fill_random(routing, sizeof(routing));
    esp_fill_random(uuid, sizeof(uuid));

    if (tesla_session_build_command(sess, (const uint8_t *)vin, strlen(vin),
                                    TESLA_DOMAIN_VEHICLE_SECURITY,
                                    payload, payload_len, routing, uuid,
                                    req, sizeof(req), &req_len,
                                    request_hash, &req_hash_len,
                                    hw_rng, NULL) != 0) {
        ESP_LOGE(TAG, "failed to sign GET_STATUS");
        return;
    }
    if (tesla_ble_send(req, req_len) != ESP_OK) {
        ESP_LOGW(TAG, "GET_STATUS send failed");
        return;
    }

    // VCSEC may emit up to three responses to one request (e.g. a WAIT/busy
    // commandStatus before the vehicleStatus result). Feed each through the
    // terminal classifier until we hit STATUS / terminal DONE / ERROR.
    //
    // Caveat: if the vehicle echoes the request's counter in every response
    // to that request, responses 2/3 carry the same counter and the anti-replay
    // check rejects them as duplicates — the loop then degrades to a single
    // response, which is perfectly fine for GET_STATUS. Confirm the counter
    // model against a real car in Phase 3.
    bool got_status = false, errored = false;
    for (int i = 0; i < 3 && !got_status && !errored; i++) {
        uint8_t resp[RX_FRAME_MAX], plain[TESLA_PB_PAYLOAD_MAX];
        size_t resp_len = 0, plain_len = 0;
        uint32_t fault = 0;
        VCSEC_FromVCSECMessage from;
        esp_err_t e;

        e = recv_for_route(resp, sizeof(resp), &resp_len, RESPONSE_TIMEOUT_MS, routing);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "no status response (attempt %d/3)", i + 1);
            break;
        }
        if (tesla_session_process_response(sess, (const uint8_t *)vin, strlen(vin),
                                           request_hash, req_hash_len,
                                           resp, resp_len,
                                           plain, sizeof(plain), &plain_len,
                                           &fault) != 0) {
            ESP_LOGW(TAG, "status response rejected (attempt %d/3)", i + 1);
            break;
        }
        if (fault != 0) {
            // Protocol-layer error (review N2): surface it distinctly. VCSEC
            // may still follow with a specific result, so keep collecting.
            ESP_LOGW(TAG, "status response fault=%u; waiting for the specific result",
                     (unsigned)fault);
        }
        if (tesla_pb_decode_vcsec_from(plain, plain_len, &from) != 0) {
            ESP_LOGW(TAG, "bad status payload (attempt %d/3)", i + 1);
            break;
        }

        switch (tesla_vcsec_ingest(&from)) {
        case TESLA_VCSEC_STATUS: {
            int presence = (int)from.sub_message.vehicleStatus.userPresence;
            int lock     = (int)from.sub_message.vehicleStatus.vehicleLockState;
            int sleep    = (int)from.sub_message.vehicleStatus.vehicleSleepStatus;
            got_status = true;
            ESP_LOGI(TAG, "status: presence=%s lock=%s sleep=%s",
                     presence_name(presence), lock_name(lock), sleep_name(sleep));
            if (presence != *last_presence || lock != *last_lock || sleep != *last_sleep) {
                ESP_LOGI(TAG, "status delta: presence %s->%s, lock %s->%s, sleep %s->%s",
                         presence_name(*last_presence), presence_name(presence),
                         lock_name(*last_lock), lock_name(lock),
                         sleep_name(*last_sleep), sleep_name(sleep));
                *last_presence = presence;
                *last_lock = lock;
                *last_sleep = sleep;
            }
            break;
        }
        case TESLA_VCSEC_DONE:
            // Terminal success without a status payload (GET_STATUS answered
            // with an empty OK): nothing to report.
            return;
        case TESLA_VCSEC_ERROR:
            errored = true;
            ESP_LOGW(TAG, "VCSEC nominalError for GET_STATUS");
            break;
        default:  // PENDING (WAIT / busy)
            ESP_LOGD(TAG, "status WAIT (response %d/3); waiting for the result", i + 1);
            break;
        }
    }
    if (!got_status && !errored) {
        ESP_LOGW(TAG, "no terminal status after 3 responses");
    }
}

static void client_task(void *arg)
{
    (void)arg;
    tesla_keypair_t key;
    tesla_car_addr_t addr;
    char vin[32];
    bool configured = false;

    while (true) {
        if (!load_config(&key, &addr, vin)) {
            if (!configured) {
                ESP_LOGI(TAG, "no enrolled Tesla key/link yet (pairing is Phase 3); waiting");
                configured = false;
            }
            vTaskDelay(pdMS_TO_TICKS(NO_KEY_DELAY_S * 1000));
            continue;
        }
        configured = true;

        ESP_LOGI(TAG, "connecting to car MAC %02X:%02X:%02X:%02X:%02X:%02X",
                 addr.val[5], addr.val[4], addr.val[3], addr.val[2],
                 addr.val[1], addr.val[0]);
        if (tesla_ble_connect(&addr, CONNECT_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "connect failed; retrying in %d s", RECONNECT_DELAY_S);
            // Tear down any half-opened/ghost link (review S3) so the next
            // cycle starts from ST_IDLE instead of wedging the state machine.
            tesla_ble_disconnect();
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_S * 1000));
            continue;
        }

        // VCSEC handshake.
        tesla_session_t sess;
        tesla_session_init(&sess, TESLA_DOMAIN_VEHICLE_SECURITY, now_ms);
        {
            uint8_t routing[16], challenge[16], req[400], resp[320];
            size_t req_len = 0, resp_len = 0;
            esp_fill_random(routing, sizeof(routing));
            esp_fill_random(challenge, sizeof(challenge));
            if (tesla_build_handshake_request(TESLA_DOMAIN_VEHICLE_SECURITY,
                                              key.pub, routing, challenge,
                                              req, sizeof(req), &req_len) != 0) {
                ESP_LOGE(TAG, "failed to build handshake request");
                goto cycle_done;
            }
            if (tesla_ble_send(req, req_len) != ESP_OK) {
                ESP_LOGW(TAG, "handshake send failed");
                goto cycle_done;
            }
            if (recv_for_route(resp, sizeof(resp), &resp_len,
                               RESPONSE_TIMEOUT_MS, routing) != ESP_OK) {
                ESP_LOGW(TAG, "no handshake response");
                goto cycle_done;
            }
            if (tesla_session_handshake(&sess, &key, (const uint8_t *)vin, strlen(vin),
                                        challenge, resp, resp_len,
                                        hw_rng, NULL) != 0) {
                ESP_LOGW(TAG, "handshake rejected (key not enrolled?)");
                goto cycle_done;
            }
            if (!sess.whitelisted) {
                // HMAC was valid (we have K), but the car reports this key is
                // NOT on the whitelist — un-enrolled, so poll will fail.
                ESP_LOGW(TAG, "handshake OK, but key NOT on whitelist "
                              "(enroll via Phase 3 pairing)");
                goto cycle_done;
            }
            ESP_LOGI(TAG, "VCSEC handshake complete (epoch=counter=%u)",
                     (unsigned)sess.counter);
        }

        // GET_STATUS poll while connected; log presence/lock/sleep deltas.
        {
            int last_presence = -1, last_lock = -1, last_sleep = -1;
            for (int i = 0; i < POLLS_PER_CONN; i++) {
                refresh_status(&sess, vin, &last_presence, &last_lock, &last_sleep);
                vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_S * 1000));
            }
        }

    cycle_done:
        // Idle-disconnect (load-bearing, plan §6): free the car link.
        tesla_ble_disconnect();
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_S * 1000));
    }
}

esp_err_t tesla_ble_client_init(void)
{
    s_rxq = xQueueCreate(4, sizeof(rx_frame_t));
    if (s_rxq == NULL) {
        ESP_LOGE(TAG, "failed to create rx queue");
        return ESP_ERR_NO_MEM;
    }
    tesla_ble_set_rx_cb(client_rx_cb, NULL);

    if (xTaskCreate(client_task, "tesla_client", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create client task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
