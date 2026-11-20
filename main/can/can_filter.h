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
// The CAN controllers receive both buses in full. If the filter is empty,
// can_filter_should_forward() returns true for the ids dbc/used_messages.txt
// lists on that bus. If the filter has any entries, only frames whose
// (bus_id, addr) exactly match an entry are forwarded; the ids need no hardware
// filter, the only limit is the 512-byte filter write (about 125 ids).

esp_err_t can_filter_init(void);

// Replace the current filter with the rules encoded in `data`. Passing
// len == 0 clears the filter (back to the used_messages.txt ids).
esp_err_t can_filter_set(const uint8_t *data, size_t len);

// Return true if dbc/used_messages.txt lists standard id `addr` on `bus_id`: the frames the firmware itself
// works with (signal cache, automations, controls). One bit lookup, no lock.
bool can_filter_is_used(uint8_t bus_id, uint32_t addr);

// Return true if a frame on `bus_id` with arbitration id `addr` should be
// forwarded to the BLE client.
bool can_filter_should_forward(uint8_t bus_id, uint32_t addr);
