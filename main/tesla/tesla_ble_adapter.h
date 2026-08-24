/*
 * Tesla BLE adapter: NimBLE *central* connection to the vehicle-command GATT
 * service.
 *
 * Owns the entire central GAP path: a connect callback, a GATT discovery
 * flow, and the vehicle notification (indicate) handler — entirely separate
 * from the peripheral GATT server's gap_event_handler in main/ble/ble_server.c.
 * A central connection event must never feed the server's slot table.
 *
 * One car link at a time. The car's BLE address comes from app provisioning
 * (app-channel opcode 0x04, persisted in NVS) — this module never scans.
 * Messages are framed with the 2-byte big-endian length prefix the vehicle
 * expects.
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

// Largest vehicle frame the RX path materializes. The NimBLE adapter can
// assemble a payload of roughly 598 bytes, so keep a little headroom while
// using one bound for the adapter, pairing/client queues, and task buffers.
// This is deliberately a frame-only bound; the adapter allocates two extra
// bytes for the big-endian length prefix.
#define TESLA_RX_FRAME_MAX 600

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

// Keep a quiet link alive (e.g. the enrollment tap window): a GATT read
// resets the supervision timeout without injecting data. Fire from a task;
// no-op when not connected.
esp_err_t tesla_ble_keepalive(void);

// Terminate the central link (error / end of use). Safe to call when not
// connected.
void tesla_ble_disconnect(void);

#ifdef __cplusplus
}
#endif
