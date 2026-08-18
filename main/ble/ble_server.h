#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

// Initialize NimBLE stack and GATT server
esp_err_t ble_server_init(void);

// Start advertising
esp_err_t ble_server_start(void);

// Send a batch of CAN frames as a BLE notification.
// Data must be in DashPilot wire format: [count][bus,addr_LE32,len,data...]...
// Returns ESP_OK if notification was sent, ESP_ERR_INVALID_STATE if not connected.
esp_err_t ble_server_notify(const uint8_t *data, size_t len);

// Check if a client is connected and subscribed to notifications
bool ble_server_is_connected(void);

// Get the current BLE connection handle (BLE_HS_CONN_HANDLE_NONE if not connected)
uint16_t ble_server_get_conn_handle(void);

// Open a pairing window so one new (not-yet-bonded) device can pair. Called when
// an already-paired device requests it. The current connection (normally the
// requester) is dropped to free the single connection slot; the requesting app
// pauses its auto-reconnect for the window so the new device can take the slot.
// The window auto-closes after a timeout or once a new device pairs. With no
// bonds yet the device is always pairable, so this is only meaningful once at
// least one device is bonded.
void ble_server_enter_pairing_mode(void);

// Wipe all bonds, rotate to a fresh address, and reboot. Does not return.
void ble_server_factory_reset(void);
