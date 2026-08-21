/*
 * Tesla BLE pairing — Phase 3 enrollment (one-time physical, in-car).
 *
 * Generates a fresh NIST-P256 keypair and sends a present-key
 * `addKeyToWhitelistAndAddPermissions` request to the car over the central BLE
 * link, then persists the enrolled key once the car confirms it. The request is
 * not cryptographically signed — the car only commits the key after the owner
 * taps an NFC card on the center console and approves on the touchscreen, so
 * this is an interactive, user-present flow, not something that runs unattended
 * (mirroring the reference SendAddKeyRequestWithRole).
 *
 * Role is CHARGING_MANAGER (read + charge only) per the plan; DRIVER opt-in is
 * Phase 5. The trigger is the Phase 4 app channel (or a future console
 * command) calling tesla_pairing_configure(); the task then runs the
 * enrollment and hands off to the Phase 2/3 client poll loop.
 *
 * The enrolled private key is plaintext in NVS (release blocker, plan §7);
 * never log any key material.
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

// Provision the target VIN (17 chars) + car BLE address and wake the pairing
// task to enroll. This is the firmware half of the Phase 4 app pairing UX.
esp_err_t tesla_pairing_configure(const char *vin, const tesla_car_addr_t *addr);

// Auto-provision hook driven by the observer (no console/app input needed): the
// observer discovers the car's BLE address straight from the scan, so as soon
// as the advertisement name identifies THIS board's target vehicle it can arm
// the pairing task with the VIN + discovered address and enroll unattended.
bool tesla_pairing_is_target_vehicle(const char *name, size_t name_len);
esp_err_t tesla_pairing_observe_vehicle(const char *name, size_t name_len,
                                        const tesla_car_addr_t *addr);

#ifdef __cplusplus
}
#endif
