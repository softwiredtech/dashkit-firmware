/*
 * Tesla BLE pairing — present-key enrollment (one-time physical, in-car).
 *
 * Generates a fresh NIST-P256 keypair and sends a present-key
 * `addKeyToWhitelistAndAddPermissions` request to the car over the central BLE
 * link, then persists the enrolled key once the car confirms it. The request is
 * not cryptographically signed — the car only commits the key after the owner
 * taps an NFC card on the center console and approves on the touchscreen, so
 * this is an interactive, user-present flow (mirroring the reference
 * SendAddKeyRequestWithRole).
 *
 * Enrolled role is CHARGING_MANAGER (read + charge only). Provisioning (VIN +
 * car BLE address) arrives from the app via tesla_pairing_configure(), called
 * by the app-channel handler; enrollment itself starts only on the app's
 * explicit 0x01 command and hands off to the client poll loop on success.
 *
 * The enrolled private key is plaintext in NVS (release blocker); never log
 * any key material.
 */

#pragma once

#include "esp_err.h"

#include "crypto.h"
#include "tesla_ble_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

// Spawn the pairing task. Idles when a key is already enrolled; otherwise
// waits for VIN + car address to be provisioned via tesla_pairing_configure()
// and then performs the enrollment. Safe to call once at boot (feature-gated).
esp_err_t tesla_pairing_init(void);

// Provision the target VIN (17 chars) + car BLE address and stage the car
// (TESLA_LINK_STAGED). This is the firmware half of the app pairing UX: the
// app scans for the vehicle, then sends app-channel opcode 0x04 with the
// VIN + MAC it found. Never begins enrollment on its own.
esp_err_t tesla_pairing_configure(const char *vin, const tesla_car_addr_t *addr);

// ---- app-channel control (app-triggered-only pairing) ----
//
// Enrollment starts ONLY on an explicit app command (0x01); the pairing task
// simply waits here while a car is staged.

// Begin enrollment for the staged car. No-op / errors if no car is staged or a
// key is already enrolled.
esp_err_t tesla_pairing_start(void);

// Cancel an in-progress (open tap-window) enrollment; returns to staged.
esp_err_t tesla_pairing_cancel(void);

// Factory-reset Tesla state (erase the enrolled key). The app provisions the
// car again (opcode 0x04) before the next enrollment.
esp_err_t tesla_pairing_reset(void);

#ifdef __cplusplus
}
#endif
