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

// Enter pairing mode for `duration_sec` seconds.
// During this window, new devices can bond. The green LED blinks to indicate
// pairing mode. After the window expires, pairing requests are rejected.
void ble_server_enter_pairing_mode(uint32_t duration_sec);
