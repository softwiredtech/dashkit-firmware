/*
 * Tesla BLE storage — NVS persistence for the vehicle-command client state.
 *
 * Namespace "tesla" holds the pieces the Phase 2/3 central client needs across
 * reboots so it does not re-pair on every power cycle:
 *   - the enrolled client keypair (private + public scalar/point, raw bytes)
 *   - the 17-char VIN (personalization for every command)
 *   - the car's BLE address (so we connect directly instead of re-scanning)
 *
 * Phase 3 (pairing) writes these from its enrollment flow (tesla_pairing.c);
 * the Phase 2/3 client reads them (tesla_storage_load_*) so that once a key is
 * enrolled the handshake + GET_STATUS poll can run. Key generation/re-enrollment
 * land here too. (Session caching is deferred — the boot-relative clock cannot
 * carry the vehicle-clock offset across a reboot, so the client re-handshakes
 * each cycle; see the plan §Phase 3.)
 *
 * Matches the plan: "plaintext NVS private key initially (matches the ESPHome
 * reference)" — no key material is ever logged; flash-encryption / SE hardening
 * is the documented release-blocker risk (plan §7).
 *
 * RELEASE BLOCKER (do not ship a DRIVER-role build with this): the private key
 * lives plaintext in NVS. Flash encryption and/or a secure-element-backed key
 * must land before any production/DRIVER-role release (plan §7 / ADR §). See
 * plan §7 "release-blocker candidate".
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

// BLE address the client connects to (fields match ble_addr_t layout: a type
// byte followed by the 6 MAC bytes, big-endian).
typedef struct {
    uint8_t type;
    uint8_t val[6];
} tesla_car_addr_t;

// --- keypair ---
bool tesla_storage_has_key(void);
esp_err_t tesla_storage_load_key(tesla_keypair_t *key);
esp_err_t tesla_storage_save_key(const tesla_keypair_t *key);

// --- VIN (17 chars, NUL-terminated) ---
esp_err_t tesla_storage_load_vin(char *vin, size_t cap);
esp_err_t tesla_storage_save_vin(const char *vin);

// --- car BLE address ---
esp_err_t tesla_storage_load_car_addr(tesla_car_addr_t *addr);
esp_err_t tesla_storage_save_car_addr(const tesla_car_addr_t *addr);

// ---- Onboard advertisement-name log (in-car diagnostic) ----
//
// Records EVERY distinct BLE local name the observer sees (not just Tesla
// matches), so a bench run near a car made WITHOUT a live serial monitor can
// be read off at the next boot and we can see exactly what the car (and other
// nearby devices) actually broadcast. Entries are dedup by name bytes. This is
// a lightweight diagnostic aid, not a telemetry subsystem.
#define TESLA_ADVERT_LOG_MAX 64
typedef struct {
    uint8_t  name[32];
    uint8_t  name_len;
    uint8_t  format;      // tesla_name_format (0=other, 1=legacy, 2=modern)
    uint8_t  matched;     // 1 if the name matched a Tesla format
    uint8_t  mac[6];
    int8_t   rssi;        // dBm (last sighting)
    uint8_t  _pad;
    uint16_t count;       // number of sightings this boot+
    uint32_t time_s;      // seconds since boot on first sighting
} __attribute__((packed)) tesla_advert_log_entry_t;

void tesla_advert_log_add(const uint8_t *name, size_t name_len, uint8_t matched,
                          uint8_t format, const uint8_t mac[6], int8_t rssi);
void tesla_advert_log_dump(void);

// Persist and return a monotonically increasing boot counter. Confirms the
// board actually powered up during an unattended run (e.g. in the car): it
// advances every boot regardless of BLE or advert-log writes, so a missing
// run shows up as a gap in the sequence when reading the logs back.
uint32_t tesla_storage_boot_count(void);

#ifdef __cplusplus
}
#endif
