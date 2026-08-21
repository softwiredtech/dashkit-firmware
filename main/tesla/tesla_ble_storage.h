/*
 * Tesla BLE storage — NVS persistence for the vehicle-command client state.
 *
 * Namespace "tesla" holds the pieces the Phase 2/3 central client needs across
 * reboots so it does not re-pair on every power cycle:
 *   - the enrolled client keypair (private + public scalar/point, raw bytes)
 *   - the 17-char VIN (personalization for every command)
 *   - the car's BLE address (so we connect directly instead of re-scanning)
 *
 * Phase 3 (pairing) fills these via tesla_storage_save_*; Phase 2 code reads
 * them (tesla_storage_load_*) so that once a key is enrolled the handshake +
 * GET_STATUS poll can run. Session caching (epoch/counter/clock offset) and
 * key generation/re-enrollment land in Phase 3.
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

// Remove all Tesla state (used by a factory reset / re-pair flow in Phase 3+).
void tesla_storage_erase_all(void);

#ifdef __cplusplus
}
#endif
