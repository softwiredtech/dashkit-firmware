/*
 * DashPilot app-channel GATT service (CADA02xx) — Phase 4.
 *
 * Exposes the DashKit's Tesla BLE state to the phone app over the existing
 * phone<->DashKit bond (link 1), WITHOUT touching the phone<->DashKit pairing
 * itself or the CAN->BLE path. Because the DashKit is installed inside the car
 * trim, its physical LEDs are not user-visible, so this service is the sole
 * surface for Tesla status and pairing control.
 *
 *   Service  CADA0200-CA00-B1E0-B0D6-C000AA0100A1
 *   Command  CADA0201 (write, encrypted)   [opcode][value_lo][value_hi?]
 *   Status   CADA0202 (notify, encrypted)  see report_status frame
 *
 * Pairing is app-triggered only (decided 2026-08-20): the firmware observer may
 * stage a discovered car (link_state TESLA_LINK_STAGED) but the pairing task
 * launches only on TESLA_CMD_START and cancels on TESLA_CMD_CANCEL. See
 * tesla-ble-app-ux-handoff.md / tesla-ble-integration-plan.md Phase 4.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---- link_state (status byte 1) ----
#define TESLA_LINK_NEVER_ENROLLED          0x00
#define TESLA_LINK_ENROLLED_NOT_CONNECTED  0x01
#define TESLA_LINK_ENROLLED_CONNECTED      0x02
#define TESLA_LINK_PAIRING_WINDOW          0x03
#define TESLA_LINK_ENROLLMENT_FAULT        0x04
#define TESLA_LINK_STAGED                  0x05   // car found, awaiting app start

// ---- fault detail (status byte 6; 0xFF unless link_state == 0x04) ----
#define TESLA_FAULT_NONE       0xFF
#define TESLA_FAULT_TAP_TIMEOUT 0x00   // tap window expired
#define TESLA_FAULT_REJECTED    0x01   // rejected (whitelistOperationInformation)
#define TESLA_FAULT_PROTOCOL    0x02   // protocol / signed_message_fault
#define TESLA_FAULT_PERSIST     0x03   // persistence failure
#define TESLA_FAULT_LINK        0x04   // link / send failure

// ---- command opcodes (write CADA0201) ----
#define TESLA_CMD_START   0x01   // start / retry enrollment (app-only trigger)
#define TESLA_CMD_RESET   0x02   // factory-reset Tesla state (erase key)
#define TESLA_CMD_CANCEL  0x03   // cancel an open pairing window

// Register the service in the GATT table (always built, like ble_ota). Called
// after ble_server_init().
esp_err_t ble_appchan_init(void);

// The GATT blob for the app-channel service, appended into s_gatt_svcs[].
const struct ble_gatt_svc_def *ble_appchan_get_service_def(void);

// Push a Tesla status frame to the subscribed (active) phone. Safe to call from
// any task once the stack is synced. `fault_detail` is ignored unless
// `link_state == TESLA_LINK_ENROLLMENT_FAULT`.
//
// Frame (notified on change — both this firmware and the Android app use GATT
// notifications, CCCD 0x0001):
//   [0] frame version = 0x01
//   [1] link_state
//   [2] presence           0 absent, 1 present, 0xFF unknown
//   [3] lock               0 unlocked, 1 locked, 0xFF unknown
//   [4] sleep              0 awake, 1 asleep, 0xFF unknown
//   [5] flags              bit0 charge-connected, bit1 charging, bit2 climate-on
//   [6] fault detail       (0xFF unless link_state == TESLA_LINK_ENROLLMENT_FAULT)
void ble_appchan_report_status(uint8_t link_state, uint8_t presence, uint8_t lock,
                               uint8_t sleep, uint8_t flags, uint8_t fault_detail);

#ifdef __cplusplus
}
#endif
