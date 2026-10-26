#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialize NimBLE stack and GATT server
esp_err_t ble_server_init(void);

// Start advertising
esp_err_t ble_server_start(void);

// Send a batch of CAN frames as a BLE notification.
// Data: [count][timestamp_us_LE32,bus,addr_LE32,len,data...]...
// Fans out to both encrypted subscribers, splitting batches for each MTU.
// Returns ESP_OK if any notification was sent, INVALID_STATE without receivers.
esp_err_t ble_server_notify(const uint8_t *data, size_t len);

// Check if a client is connected and subscribed to notifications
bool ble_server_is_connected(void);

// Encrypted clients may write unless they set their read-only flag.
// Clients can always send PING or set their own BLE_READ_ONLY connection flag.
bool ble_server_can_write(uint16_t conn_handle);

// Open a pairing window so one new (not-yet-bonded) device can pair. Called when
// an already-paired device requests it. A free slot is used when available;
// otherwise another connection is disconnected to make room (preserving OTA).
// The window auto-closes after a timeout or once a new device pairs. With no
// bonds yet the device is always pairable, so this is only meaningful once at
// least one device is bonded.
void ble_server_enter_pairing_mode(void);

// Wipe all bonds, rotate to a fresh address, and reboot. Does not return.
void ble_server_factory_reset(void);
