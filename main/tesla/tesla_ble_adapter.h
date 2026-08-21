/*
 * Tesla BLE adapter: NimBLE *central* connection to the vehicle-command GATT
 * service, plus the Phase 1 observer scan.
 *
 * Owns the entire central GAP path (ADR 0001 review note 2): a scan callback,
 * a connect callback, a GATT discovery flow, and the vehicle notification
 * (indicate) handler live here — entirely separate from the peripheral GATT
 * server's gap_event_handler in main/ble/ble_server.c. A central connection
 * event must never feed the server's slot table.
 *
 * One car link is supported at a time; the client connects → exchanges → sends
 * → disconnects (idle-disconnect is load-bearing, plan §6). Messages are
 * framed with the 2-byte big-endian length prefix the vehicle expects.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Callback invoked for each complete length-prefixed frame received from the
// vehicle (i.e. the RoutableMessage bytes, with the framing stripped).
typedef void (*tesla_ble_rx_fn_t)(const uint8_t *data, size_t len, void *arg);

// Starts the Phase 1 observer scan (logs nearby Tesla advertisements). Safe to
// call once at boot; the scan runs continuously and is independent of the
// central connect path.
esp_err_t tesla_ble_adapter_observer_init(void);

// Registers the frame receive callback used by the central connection.
void tesla_ble_set_rx_cb(tesla_ble_rx_fn_t cb, void *arg);

// Blocking connect to the car at `addr` (tesla_car_addr_t layout: type + 6
// MAC bytes). Discovers the vehicle service + write/indicate characteristics,
// exchanges MTU, subscribes to 0213, and returns once ready or after
// timeout_ms. Callable from a task (not from a NimBLE callback).
esp_err_t tesla_ble_connect(const void *addr, uint32_t timeout_ms);

// Send `data` (a RoutableMessage, unframed) to the vehicle, chunking to the
// negotiated ATT MTU with the 2-byte length prefix. Returns after the last
// chunk is written.
esp_err_t tesla_ble_send(const uint8_t *data, size_t len);

// Terminate the central link (idle-disconnect / error). Safe to call when not
// connected.
void tesla_ble_disconnect(void);

// True while a central connection is established.
bool tesla_ble_is_connected(void);

#ifdef __cplusplus
}
#endif
