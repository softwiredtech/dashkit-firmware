/*
 * Tesla BLE client — high-level vehicle-command facade + poll loop.
 *
 * Orchestrates connect → VCSEC handshake → persistent GET_STATUS poll (the
 * link stays up; matches esphome-tesla-ble). The low-level central GATT
 * transport lives in tesla_ble_adapter; this module owns the protocol session
 * (tesla_session_t), the response correlation, and the app-channel status
 * reporting. Gated by CONFIG_DASHKIT_TESLA_BLE so a feature-off build drops
 * it entirely.
 */

#pragma once

#include "esp_err.h"
#include "tesla_ble_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Starts the client task (spawns the poll loop). Safe to call once at boot
// when CONFIG_DASHKIT_TESLA_BLE is enabled; with no enrolled key/link it logs
// a canary and waits.
esp_err_t tesla_ble_client_init(void);

// One received Tesla frame, as queued by an RX callback.
typedef struct {
    uint16_t len;
    uint8_t  data[TESLA_RX_FRAME_MAX];
} tesla_rx_frame_t;

// Wait up to timeout_ms on `q` for a frame whose to_destination.routing_address
// matches `routing` (the address used for the outstanding request), so a late
// frame from a previous request never answers the current one. Other frames
// are dropped. `item` is the caller's task-owned receive buffer.
esp_err_t tesla_recv_for_route(QueueHandle_t q, tesla_rx_frame_t *item,
                               uint8_t *buf, size_t cap, size_t *out_len,
                               uint32_t timeout_ms, const uint8_t routing[16]);

#ifdef __cplusplus
}
#endif
