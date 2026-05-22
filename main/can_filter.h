#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// CAN frame filter (whitelist of (bus_id, addr) pairs).
//
// Wire format consumed by can_filter_set():
//   [bus_id : 1 byte]
//   [num_addrs : 1 byte]
//   [addr1 : 4 bytes LE]
//   [addr2 : 4 bytes LE]
//   ...
// repeated for each bus.
//
// If the filter is empty, can_filter_should_forward() returns true for all
// frames (i.e. no filtering). If the filter has any entries, only frames
// whose (bus_id, addr) exactly match an entry are forwarded.

esp_err_t can_filter_init(void);

// Replace the current filter with the rules encoded in `data`. Passing
// len == 0 clears the filter (forward everything).
esp_err_t can_filter_set(const uint8_t *data, size_t len);

// Return true if a frame on `bus_id` with arbitration id `addr` should be
// forwarded to the BLE client.
bool can_filter_should_forward(uint8_t bus_id, uint32_t addr);
