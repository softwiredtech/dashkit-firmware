/*
 * Tesla BLE client — high-level vehicle-command facade + poll loop.
 *
 * Orchestrates the Phase 2 in-car milestone: connect → VCSEC handshake →
 * GET_STATUS poll → disconnect (idle-disconnect). The low-level central GATT
 * transport lives in tesla_ble_adapter; this module owns the protocol session
 * (tesla_session_t), the response correlation, and the status logging. It is
 * gated by CONFIG_DASHKIT_TESLA_BLE so a feature-off build drops it entirely.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Starts the client task (spawns the poll loop). Safe to call once at boot
// when CONFIG_DASHKIT_TESLA_BLE is enabled. If no enrolled key/link exists yet
// (Phase 3), the loop logs a canary and waits.
esp_err_t tesla_ble_client_init(void);

#ifdef __cplusplus
}
#endif
