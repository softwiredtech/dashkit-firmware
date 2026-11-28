#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// CAN frame filters: per connected device, a whitelist of (bus_id, addr) pairs.
//
// Wire format consumed by can_filter_set():
//   [bus_id : 1 byte]
//   [num_addrs : 1 byte]
//   [addr1 : 4 bytes LE]
//   [addr2 : 4 bytes LE]
//   ...
// repeated for each bus.
//
// The CAN controllers receive both buses in full. Each connected device gets
// the frames whose (bus_id, addr) exactly match an entry of its own list; the
// ids need no hardware filter, the only limit is the 512-byte filter write
// (about 125 ids). A device that wrote no list gets the ids
// dbc/used_messages.txt lists. can_filter_should_forward() is the union of
// every connected device's set (the used_messages.txt ids with no device).

// Connection slots, numbered by the BLE server (one per possible connection).
#define CAN_FILTER_SLOTS  8

esp_err_t can_filter_init(void);

// A device connected in `slot`: it gets the used_messages.txt ids until it
// writes its own list.
void can_filter_open(uint8_t slot);

// The device in `slot` disconnected: its list no longer counts.
void can_filter_close(uint8_t slot);

// Replace the list of the device in `slot` with the rules encoded in `data`.
// Passing len == 0 clears it (back to the used_messages.txt ids).
esp_err_t can_filter_set(uint8_t slot, const uint8_t *data, size_t len);

// Return true if dbc/used_messages.txt lists standard id `addr` on `bus_id`: the frames the firmware itself
// works with (signal cache, automations, controls). One bit lookup, no lock.
bool can_filter_is_used(uint8_t bus_id, uint32_t addr);

// Return true if any connected device gets a frame on `bus_id` with
// arbitration id `addr` (so it should be forwarded to the BLE side).
bool can_filter_should_forward(uint8_t bus_id, uint32_t addr);

// Return true if the device in `slot` gets that frame.
bool can_filter_wants(uint8_t slot, uint8_t bus_id, uint32_t addr);
