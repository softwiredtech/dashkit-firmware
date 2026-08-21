/*
 * Tesla BLE pairing — Phase 3 enrollment implementation.
 *
 * Enrollment is a one-time physical flow: the board generates a fresh P-256
 * keypair and sends a present-key addKeyToWhitelistAndAddPermissions request
 * over the central BLE link. The request is NOT cryptographically signed — the
 * car authorizes it physically: the owner taps an NFC card on the center
 * console and confirms on the touchscreen. So the pairing task keeps reading
 * responses until VCSEC reports the whitelist operation completed (or the tap
 * window times out), then persists the key.
 *
 * The enroll path reuses the tesla_ble_adapter central transport (connect,
 * length-framed send, frame RX callback); it does not touch the peripheral
 * server's GAP handler. On success the keypair + VIN + car address are
 * persisted so the Phase 2/3 client poll loop can handshake + GET_STATUS.
 */

#include "tesla_pairing.h"

#include "tesla_ble_adapter.h"
#include "led.h"
#include "protobuf_build.h"
#include "universal_message.pb.h"
#include "vcsec.pb.h"

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "tesla_pairing";

// Auto-provision target for the observer hook (handoff "Option B"): the car
// this board is being enrolled against. A real Tesla's legacy local name is
// "S" + first-16-hex-chars(SHA1(VIN)) + role letter (CHARGING_MANAGER here).
// Matching the derived name — not "any Tesla" — ensures we only auto-enroll
// against OUR car, never a random Tesla in range. Mapping verified on-air for
// this VIN (handoff note): 5YJ3E1EB3MF074051 -> Sf9cd80ddffdd5492C.
#define TESLA_TARGET_VIN  "5YJ3E1EB3MF074051"
#define TESLA_TARGET_NAME "Sf9cd80ddffdd5492C"

// Enrolled role + form factor for the DashPilot key (plan §3): read + charge
// only, presented as an Android-style device. DRIVER opt-in is Phase 5.
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

#define RX_FRAME_MAX 320

typedef struct {
    uint16_t len;
    uint8_t  data[RX_FRAME_MAX];
} pairing_frame_t;

static QueueHandle_t s_rxq;

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
    if (s_rxq == NULL || len == 0 || len > RX_FRAME_MAX) {
        return;
    }
    pairing_frame_t f;
    f.len = (uint16_t)len;
    memcpy(f.data, data, len);
    xQueueSend(s_rxq, &f, 0);
}

static esp_err_t pairing_recv(uint8_t *buf, size_t cap, size_t *out_len,
                              uint32_t timeout_ms)
{
    pairing_frame_t f;

    if (xQueueReceive(s_rxq, &f, (TickType_t)pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (f.len > cap) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(buf, f.data, f.len);
    if (out_len != NULL) {
        *out_len = f.len;
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

    // nanopb only writes a which_* selector when that member is on the wire;
    // zero the structs so a WAIT reply (commandStatus with no sub_status) is
    // read as "keep waiting" rather than as uninitialized stack garbage.
    memset(&rm, 0, sizeof(rm));
    memset(&from, 0, sizeof(from));

    // The car replies with a RoutableMessage whose protobuf_message_as_bytes
    // holds the VCSEC.FromVCSECMessage (plaintext — no session, no encryption).
    if (tesla_pb_decode_routable(frame, len, &rm) == 0 &&
        rm.which_payload == (pb_size_t)UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag) {
        payload = rm.payload.protobuf_message_as_bytes.bytes;
        plen = rm.payload.protobuf_message_as_bytes.size;
    } else if (rm.has_signedMessageStatus &&
               rm.signedMessageStatus.signed_message_fault !=
                   UniversalMessage_MessageFault_E_MESSAGEFAULT_ERROR_NONE) {
        // Protocol-layer rejection (e.g. INVALID_SIGNATURE): nothing to retry
        // around — surface it for diagnosis and stop.
        ESP_LOGW(TAG, "enrollment rejected at protocol layer (signed_message_fault=%d)",
                 (int)rm.signedMessageStatus.signed_message_fault);
        return -1;
    }

    if (tesla_pb_decode_vcsec_from(payload, plen, &from) != 0) {
        return 0;
    }
    if (from.which_sub_message != (pb_size_t)VCSEC_FromVCSECMessage_commandStatus_tag) {
        return 0;   // vehicleStatus / empty — not the whitelist result yet
    }
    const VCSEC_CommandStatus *cs = &from.sub_message.commandStatus;
    if (cs->which_sub_message != (pb_size_t)VCSEC_CommandStatus_whitelistOperationStatus_tag) {
        return 0;   // OPERATIONSTATUS_WAIT (awaiting tap) — keep waiting
    }
    const VCSEC_WhitelistOperation_status *ws = &cs->sub_message.whitelistOperationStatus;
    if (ws->whitelistOperationInformation ==
            VCSEC_WhitelistOperation_information_E_WHITELISTOPERATION_INFORMATION_NONE) {
        return 1;
    }
    if (ws->whitelistOperationInformation ==
        VCSEC_WhitelistOperation_information_E_WHITELISTOPERATION_INFORMATION_ATTEMPTING_TO_ADD_KEY_THAT_IS_ALREADY_ON_THE_WHITELIST) {
        // A prior attempt enrolled this same key but we lost the confirm
        // response; the key is already on the car, so this is success.
        return 1;
    }
    if (info_out != NULL) {
        *info_out = (uint32_t)ws->whitelistOperationInformation;
    }
    return -1;
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

    uint8_t msg[320];
    size_t msg_len = 0;
    if (tesla_pb_build_enrollment(key, ENROLL_ROLE, ENROLL_FORM_FACTOR,
                                  msg, sizeof(msg), &msg_len) != 0) {
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

    uint32_t start = xTaskGetTickCount();
    uint32_t last_ka = 0;
    uint32_t info = 0;
    esp_err_t res = ESP_ERR_TIMEOUT;
    while ((uint32_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(TAP_TIMEOUT_MS)) {
        uint8_t frame[RX_FRAME_MAX];
        size_t flen = 0;
        if (pairing_recv(frame, sizeof(frame), &flen, RESPONSE_TIMEOUT_MS) != ESP_OK) {
            // Keep the link alive while the car silently awaits the tap: a GATT
            // read every ~4 s generates ATT traffic that resets the connection
            // supervision timer, so a quiet-but-awake car can't drop the link in
            // the middle of the tap window (established on-car: reason 0x208).
            uint32_t now = xTaskGetTickCount();
            if (now - last_ka >= pdMS_TO_TICKS(4000)) {
                last_ka = now;
                tesla_ble_keepalive();
            }
            continue;   // still waiting for the owner's tap
        }
        int r = pairing_ingest(frame, flen, &info);
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
        ESP_LOGD(TAG, "car awaiting card tap (OPERATIONSTATUS_WAIT)");
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
static bool s_configured;

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

bool tesla_pairing_is_target_vehicle(const char *name, size_t name_len)
{
    const size_t want = strlen(TESLA_TARGET_NAME);
    return name != NULL && name_len == want &&
           memcmp(name, TESLA_TARGET_NAME, want) == 0;
}

// Observer <> pairing handoff for unattended enrollment. Runs on the NimBLE
// host task (discovery callback); only writes the one-shot provisioning state,
// so the cross-task write to s_configured is benign (a plain bool, and once set
// it short-circuits every later sighting).
esp_err_t tesla_pairing_observe_vehicle(const char *name, size_t name_len,
                                        const tesla_car_addr_t *addr)
{
    if (name == NULL || addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (tesla_storage_has_key()) {
        return ESP_OK;            // already enrolled; client owns the link
    }
    if (s_configured) {
        return ESP_OK;            // already provisioned this boot
    }
    if (!tesla_pairing_is_target_vehicle(name, name_len)) {
        return ESP_ERR_NOT_FOUND; // not our target car
    }
    ESP_LOGI(TAG, "observer: target vehicle in range; auto-provisioning VIN %s",
             TESLA_TARGET_VIN);
    return tesla_pairing_configure(TESLA_TARGET_VIN, addr);
}

static void pairing_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (tesla_storage_has_key()) {
            // Already enrolled; the client poll loop owns the link. Idle until
            // a re-pair / factory reset clears the key.
            vTaskDelay(pdMS_TO_TICKS(CONFIG_WAIT_MS));
            continue;
        }

        if (!s_configured) {
            // VIN + address were provisioned into NVS by a prior successful
            // enrollment (or the Phase 4 app channel) — resume without a new
            // provision call. (configure() itself is in-RAM only.)
            if (tesla_storage_load_vin(s_vin, sizeof(s_vin)) == ESP_OK &&
                strlen(s_vin) == 17 &&
                tesla_storage_load_car_addr(&s_car_addr) == ESP_OK) {
                s_configured = true;
            } else {
                vTaskDelay(pdMS_TO_TICKS(CONFIG_WAIT_MS));
                continue;
            }
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
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_S * 1000));
            continue;
        }

        bool enrolled = false;
        for (int attempt = 0; attempt < MAX_ATTEMPTS && !enrolled; attempt++) {
            esp_err_t e = tesla_pairing_enroll(&key, s_vin, &s_car_addr);
            if (e == ESP_OK) {
                ESP_LOGI(TAG, "enrollment complete; client poll loop takes over");
                enrolled = true;
                break;
            }
            ESP_LOGW(TAG, "enrollment attempt %d/%d failed (%s); retrying in %d s",
                     attempt + 1, MAX_ATTEMPTS, esp_err_to_name(e), RETRY_DELAY_S);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_S * 1000));
        }
        if (!enrolled) {
            ESP_LOGW(TAG, "enrollment gave up after %d attempts; re-trigger via app",
                     MAX_ATTEMPTS);
        }
        s_configured = false;
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
