/*
 * DashPilot app-channel GATT service (CADA02xx).
 *
 * Exposes the DashKit's Tesla BLE state to the phone app over the existing
 * phone<->DashKit bond, without touching phone pairing or the CAN->BLE path.
 * The DashKit sits inside the car trim (its LEDs are not user-visible), so
 * this service is the sole surface for Tesla status and pairing control.
 *
 *   Service  CADA0200-CA00-B1E0-B0D6-C000AA0100A1
 *   Command  CADA0201 (write, encrypted)
 *   Status   CADA0202 (notify, encrypted)
 *
 * Pairing is app-driven end to end: the app scans for the vehicle and stages
 * it with TESLA_CMD_PROVISION, enrollment launches only on TESLA_CMD_START,
 * and TESLA_CMD_CANCEL aborts an open tap window. The firmware has no BLE
 * observer and never starts pairing on its own.
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
// Connecting → pairing: reported the moment the app's TESLA_CMD_START is
// accepted, before the DashKit has contacted the car. The car arms its NFC
// tap window only after the enrollment request is written and the VCSEC read
// returns, which can take several seconds (connect → discovery → handshake),
// so this interim state tells the app the flow is moving instead of leaving
// it on "staged" with nothing happening. Transitions index 0x06 → 0x03.
#define TESLA_LINK_CONNECTING              0x06

// ---- fault detail (status byte 6; 0xFF unless link_state == 0x04) ----
#define TESLA_FAULT_NONE       0xFF
#define TESLA_FAULT_TAP_TIMEOUT 0x00   // tap window expired
#define TESLA_FAULT_REJECTED    0x01   // rejected (whitelistOperationInformation)
#define TESLA_FAULT_PROTOCOL    0x02   // protocol / signed_message_fault
#define TESLA_FAULT_PERSIST     0x03   // persistence failure

// ---- command opcodes (write CADA0201) ----
#define TESLA_CMD_START   0x01   // start / retry enrollment (app-only trigger)
#define TESLA_CMD_RESET   0x02   // factory-reset Tesla state (erase key)
#define TESLA_CMD_CANCEL  0x03   // cancel an open pairing window

// Provision the car the app discovered: [opcode][17B VIN][1B addr type][6B MAC]
// = 25 bytes. The MAC octets are NimBLE's raw ble_addr_t.val order — val[0] is
// the LAST pair of the human-readable address ("AA:BB:CC:DD:EE:FF" -> val[5]=AA,
// val[0]=FF). Writes on this characteristic are encrypted (WRITE_ENC), so the
// VIN/MAC never cross the air in plaintext.
#define TESLA_CMD_PROVISION      0x04
#define TESLA_PROVISION_LEN      25     // 1 opcode + 17 VIN + 1 type + 6 MAC

// Register the service in the GATT table (always built, like ble_ota). Called
// after ble_server_init().
esp_err_t ble_appchan_init(void);

// The GATT blob for the app-channel service, appended into s_gatt_svcs[].
const struct ble_gatt_svc_def *ble_appchan_get_service_def(void);

// Push a Tesla status frame to the subscribed (active) phone. Safe to call from
// any task once the stack is synced. `fault_detail` is ignored unless
// `link_state == TESLA_LINK_ENROLLMENT_FAULT`.
//
// Identical consecutive frames are suppressed here (the pairing task re-reports
// its state on a fast loop), so callers don't need their own throttling for the
// link_state byte — but a connected-status frame always goes out because bytes
// 2-4 carry the changing presence/lock/sleep values.
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
