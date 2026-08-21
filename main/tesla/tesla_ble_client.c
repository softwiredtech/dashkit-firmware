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
    uint32_t deadline = (uint32_t)(xTaskGetTickCount() +
                                   pdMS_TO_TICKS(timeout_ms));

    while (true) {
        rx_frame_t f;
        uint32_t now = xTaskGetTickCount();
        uint32_t remain = (now < deadline) ? deadline - now : 0;

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
    uint8_t payload[TESLA_PB_PAYLOAD_MAX], req[400], resp[320];
    uint8_t request_hash[33], routing[16], uuid[16];
    size_t payload_len, req_len, resp_len, req_hash_len, plain_len;
    uint32_t fault = 0;
    uint8_t plain[TESLA_PB_PAYLOAD_MAX];
    VCSEC_FromVCSECMessage from;
    esp_err_t e;

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
    e = recv_for_route(resp, sizeof(resp), &resp_len, RESPONSE_TIMEOUT_MS, routing);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "no status response");
        return;
    }
    if (tesla_session_process_response(sess, (const uint8_t *)vin, strlen(vin),
                                       request_hash, req_hash_len,
                                       resp, resp_len,
                                       plain, sizeof(plain), &plain_len,
                                       &fault) != 0) {
        ESP_LOGW(TAG, "status response rejected");
        return;
    }
    if (fault != 0) {
        ESP_LOGW(TAG, "status response fault=%u", (unsigned)fault);
        return;
    }
    if (tesla_pb_decode_vcsec_from(plain, plain_len, &from) != 0) {
        ESP_LOGW(TAG, "bad status payload");
        return;
    }
    if (from.which_sub_message != VCSEC_FromVCSECMessage_vehicleStatus_tag) {
        ESP_LOGW(TAG, "unexpected VCSEC response (type=%u)", from.which_sub_message);
        return;
    }

    int presence = (int)from.sub_message.vehicleStatus.userPresence;
    int lock     = (int)from.sub_message.vehicleStatus.vehicleLockState;
    int sleep    = (int)from.sub_message.vehicleStatus.vehicleSleepStatus;
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
