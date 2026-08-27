#include "bthome.h"
#include "vehicle_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include <string.h>

static const char *TAG = "bthome";

// BTHome v2: service data (AD type 0x16) under the allocated 16-bit UUID.
#define BTHOME_UUID16       0xFCD2
#define AD_TYPE_SVC_DATA16  0x16

// Device-info byte: bit 0 = payload encrypted, bits 5-7 = BTHome version.
#define DEVINFO_ENCRYPTED   0x01
#define DEVINFO_VERSION(b)  ((b) >> 5)

#define OBJ_PACKET_ID       0x00
#define OBJ_BUTTON_EVENT    0x3A

// Button event values: 1=press 2=double 3=triple 4=long.
#define BUTTON_PRESS        0x01

// Shelly BLU buttons repeat an event burst long enough that a 30 ms window
// every 320 ms reliably catches it (the duty cycle ESPHome BLE proxies use),
// while leaving the radio mostly free for the phone link / OTA / Tesla.
#define SCAN_ITVL           0x0200  // 320 ms (0.625 ms units)
#define SCAN_WINDOW         0x0030  // 30 ms

// One press is broadcast many times with the same packet id; the id dedups the
// burst, the time floor guards pid-less senders and packet-id reuse.
#define MIN_EVENT_GAP_US    (1500 * 1000)

typedef struct {
    uint8_t addr[6];
    bool    used;
    bool    has_pid;
    uint8_t last_pid;
    int64_t last_seen_us;
    int64_t last_event_us;
} peer_t;

#define MAX_PEERS 4
static peer_t s_peers[MAX_PEERS];

static bool s_started;
static bool s_paused;
static bool s_enc_warned;

// Fixed payload size per BTHome v2 object id; -1 for ids we cannot skip over
// (objects are packed back-to-back with no per-object length).
static int obj_len(uint8_t id)
{
    switch (id) {
    case 0x04: case 0x05: case 0x0A: case 0x0B:
    case 0x42: case 0x4B: case 0xF2:
        return 3;
    case 0x3E: case 0x4C: case 0xF1:
        return 4;
    case 0x02: case 0x03: case 0x06: case 0x07: case 0x08:
    case 0x0C: case 0x0D: case 0x0E: case 0x12: case 0x13: case 0x14:
    case 0x3C: case 0x3D: case 0x3F: case 0x40: case 0x41: case 0x43:
    case 0x44: case 0x45: case 0x47: case 0x48: case 0x49: case 0x4A:
    case 0xF0:
        return 2;
    default:
        // 0x00/0x01, the 0x0F-0x30 binary sensors, 0x3A, 0x46 are all 1 byte.
        return (id <= 0x30 || id == OBJ_BUTTON_EVENT || id == 0x46) ? 1 : -1;
    }
}

// Locate the BTHome service-data payload inside raw advertising data.
static const uint8_t *find_bthome(const uint8_t *data, uint8_t len,
                                  uint8_t *out_len)
{
    uint8_t i = 0;
    while (i + 1 < len) {
        uint8_t field_len = data[i];
        if (field_len == 0 || i + 1 + field_len > len) {
            break;
        }
        if (data[i + 1] == AD_TYPE_SVC_DATA16 && field_len >= 3) {
            uint16_t uuid = data[i + 2] | ((uint16_t)data[i + 3] << 8);
            if (uuid == BTHOME_UUID16) {
                *out_len = field_len - 3;
                return &data[i + 4];
            }
        }
        i += 1 + field_len;
    }
    return NULL;
}

static peer_t *peer_get(const uint8_t *addr)
{
    peer_t *victim = &s_peers[0];
    for (int i = 0; i < MAX_PEERS; i++) {
        if (s_peers[i].used && memcmp(s_peers[i].addr, addr, 6) == 0) {
            return &s_peers[i];
        }
        if (!s_peers[i].used) {
            victim = &s_peers[i];
        } else if (victim->used &&
                   s_peers[i].last_seen_us < victim->last_seen_us) {
            victim = &s_peers[i];
        }
    }
    memset(victim, 0, sizeof(*victim));
    memcpy(victim->addr, addr, 6);
    victim->used = true;
    return victim;
}

static void handle_bthome(const uint8_t *addr, const uint8_t *p, uint8_t len)
{
    if (len < 1 || DEVINFO_VERSION(p[0]) != 2) {
        return;
    }
    if (p[0] & DEVINFO_ENCRYPTED) {
        if (!s_enc_warned) {
            s_enc_warned = true;
            ESP_LOGW(TAG, "Encrypted BTHome device seen; encryption is not "
                          "supported, disable it in the Shelly app");
        }
        return;
    }

    bool    has_pid = false;
    uint8_t pid = 0;
    int     button = -1;
    uint8_t i = 1;
    while (i < len) {
        uint8_t id = p[i++];
        int olen = obj_len(id);
        if (olen < 0 || i + olen > len) {
            break;
        }
        if (id == OBJ_PACKET_ID) {
            pid = p[i];
            has_pid = true;
        } else if (id == OBJ_BUTTON_EVENT && p[i] != 0 && button < 0) {
            button = p[i];
        }
        i += olen;
    }

    peer_t *peer = peer_get(addr);
    int64_t now = esp_timer_get_time();
    if (peer->last_seen_us == 0) {
        ESP_LOGI(TAG, "BTHome device %02X:%02X:%02X:%02X:%02X:%02X seen",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    }
    peer->last_seen_us = now;

    // Same packet id = the same event repeated within one burst.
    bool dup = has_pid && peer->has_pid && pid == peer->last_pid;
    peer->has_pid = has_pid;
    peer->last_pid = pid;
    if (button < 0 || dup) {
        return;
    }
    if (peer->last_event_us != 0 &&
        now - peer->last_event_us < MIN_EVENT_GAP_US) {
        return;
    }
    peer->last_event_us = now;

    ESP_LOGI(TAG, "Button event 0x%02X from %02X:%02X:%02X:%02X:%02X:%02X",
             button, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    if (button == BUTTON_PRESS) {
        esp_err_t err = vehicle_control_submit(VC_CMD_GLOVEBOX, 1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Glovebox submit failed: %s", esp_err_to_name(err));
        }
    }
}

static void try_disc(void);

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        uint8_t plen = 0;
        const uint8_t *p = find_bthome(event->disc.data,
                                       event->disc.length_data, &plen);
        if (p) {
            handle_bthome(event->disc.addr.val, p, plen);
        }
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        // Controller stopped the scan (cancel fires this too); restart unless
        // deliberately paused.
        try_disc();
        break;
    default:
        break;
    }
    return 0;
}

static void try_disc(void)
{
    if (!s_started || s_paused || !ble_hs_synced() || ble_gap_disc_active()) {
        return;
    }
    struct ble_gap_disc_params params = {
        .passive = 1,
        .itvl = SCAN_ITVL,
        .window = SCAN_WINDOW,
        .filter_duplicates = 0,  // event bursts reuse the address; dedup by pid
    };
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    ble_hs_id_infer_auto(0, &own_addr_type);
    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Scan start failed: %d", rc);
    }
}

void bthome_scan_start(void)
{
    s_started = true;
    s_paused = false;
    try_disc();
    ESP_LOGI(TAG, "BTHome scan started");
}

void bthome_scan_pause(void)
{
    s_paused = true;
    ble_gap_disc_cancel();  // BLE_HS_EALREADY when not scanning; harmless
}

void bthome_scan_resume(void)
{
    s_paused = false;
    try_disc();
}
