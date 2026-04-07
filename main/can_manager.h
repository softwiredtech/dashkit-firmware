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
