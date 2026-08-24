/*
 * Tesla BLE storage — NVS persistence for vehicle-command client state.
 *
 * Namespace "tesla" persists the enrolled client keypair, the 17-char VIN, and
 * the car's BLE address across reboots so the client does not re-pair on every
 * power cycle. Written by the enrollment flow (tesla_pairing.c) and read by the
 * client (tesla_storage_load_*) before each handshake.
 *
 * NOTE: the private key is stored PLAINTEXT in NVS. No key material is ever
 * logged, but flash-encryption / secure-element hardening is a RELEASE BLOCKER
 * before any production/DRIVER-role build.
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

// Erase ALL Tesla state (keypair, pub, VIN, car address). Used by the app-channel
// "reset Tesla key" command (TESLA_CMD_RESET / 0x02). After this the app
// provisions the car again (TESLA_CMD_PROVISION / 0x04) before the next
// enrollment. Never erases the phone<->DashKit BLE bonds (that is
// ble_server_factory_reset's job).
esp_err_t tesla_storage_erase_all(void);

#ifdef __cplusplus
}
#endif
