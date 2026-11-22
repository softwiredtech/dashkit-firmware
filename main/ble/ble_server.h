#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialize NimBLE stack and GATT server
esp_err_t ble_server_init(void);

// Start advertising
esp_err_t ble_server_start(void);

// A device that gets the CAN stream.
typedef struct {
    uint16_t conn;     // connection handle
    uint8_t  slot;     // its CAN filter slot (can_filter_wants)
    bool     primary;  // the first in the list: gets each batch first
} ble_receiver_t;

// The devices that get the CAN stream, primary first (see
// ble_stream_pick_primary); none during an OTA update.
size_t ble_server_receivers(ble_receiver_t *out, size_t max);

// Send a batch of CAN frames to one device as a BLE notification (at most 512 bytes).
// Data must be in DashPilot wire format: [count][bus,addr_LE32,len,data...]...
// Returns ESP_OK if the notification was queued, ESP_ERR_NO_MEM while the device's
// earlier packets are still waiting (each device has a few buffers of its own),
// ESP_ERR_INVALID_STATE during an OTA update.
esp_err_t ble_server_notify_to(const ble_receiver_t *r, const uint8_t *data, size_t len);

// Largest notification this connection carries in one piece (ATT_MTU - 3); 0 if it is gone.
size_t ble_server_payload(uint16_t conn);

// True if this connection may write (encrypted and not in read-only mode): app channel and firmware update.
bool ble_server_can_write(uint16_t conn);

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
