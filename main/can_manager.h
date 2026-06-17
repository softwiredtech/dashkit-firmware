#pragma once

#include "can_interface.h"
#include "esp_err.h"

#define CAN_MANAGER_MAX_INTERFACES  4

// Initialize the CAN manager (creates the shared frame queue)
esp_err_t can_manager_init(void);

// Register a CAN interface with the manager. Takes ownership of lifecycle.
esp_err_t can_manager_add_interface(can_interface_t *iface);

// Initialize and start all registered interfaces
esp_err_t can_manager_start(void);

// Stop all interfaces
esp_err_t can_manager_stop(void);

// Get the next received frame (blocks up to timeout_ms). Returns ESP_ERR_TIMEOUT if none.
esp_err_t can_manager_receive(can_tagged_frame_t *frame, uint32_t timeout_ms);

// Inject a frame into the RX queue (for simulator use). Timestamps automatically.
esp_err_t can_manager_inject(const can_tagged_frame_t *frame);

// Transmit a frame on the interface with the given bus_id.
// Returns ESP_ERR_NOT_FOUND if no interface matches bus_id.
esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame);

// Register (bus_id, id) so the manager keeps a snapshot of its most recent
// frame. Call once at startup for any frame you want to read-modify-write.
// Returns ESP_ERR_NO_MEM if the watch table is full.
esp_err_t can_manager_watch_frame(uint8_t bus_id, uint32_t id);

// Copy the most recently received frame for a watched (bus_id, id) into `out`.
// Returns ESP_ERR_NOT_FOUND if the id isn't watched or none has arrived yet.
esp_err_t can_manager_get_frame(uint8_t bus_id, uint32_t id, can_frame_t *out);
