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

// Observer hook for app-triggered-only enrollment: the observer discovers the
// car's BLE address straight from the scan, and when the advertisement name
// identifies THIS board's target vehicle it stages it (reports
// TESLA_LINK_STAGED / 0x05) for the app to start. It never begins enrollment
// on its own — the app's tesla_pairing_start() (0x01) is the only trigger.
bool tesla_pairing_is_target_vehicle(const char *name, size_t name_len);
esp_err_t tesla_pairing_observe_vehicle(const char *name, size_t name_len,
                                        const tesla_car_addr_t *addr);

// ---- Phase 4 app-channel control (app-triggered-only pairing) ----
//
// The DashKit sits in the car trim (LEDs not user-visible), so enrollment
// starts ONLY on an explicit app command. The observer may stage a discovered
// car (link_state TESLA_LINK_STAGED), but the pairing task waits here until
// tesla_pairing_start() is called via the app-channel command 0x01.

// Begin enrollment for the staged car. No-op / errors if no car is staged or a
// key is already enrolled.
esp_err_t tesla_pairing_start(void);

// Cancel an in-progress (open tap-window) enrollment; returns to staged.
esp_err_t tesla_pairing_cancel(void);

// Factory-reset Tesla state (erase the enrolled key). The next car sighting
// re-stages (TESLA_LINK_STAGED) and enrollment waits for an app start again.
esp_err_t tesla_pairing_reset(void);

#ifdef __cplusplus
}
#endif
