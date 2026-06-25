#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define CAN_MAX_DATA_LEN  64  // CAN FD supports up to 64 bytes

typedef struct {
    uint32_t id;           // Arbitration ID (11-bit standard or 29-bit extended)
    uint8_t  data[CAN_MAX_DATA_LEN];
    uint8_t  dlc;          // Data Length Code
    bool     extended;     // true = 29-bit extended ID
    bool     fd;           // true = CAN FD frame
    bool     brs;          // true = bit rate switch (CAN FD)
} can_frame_t;

// Frame tagged with the bus it was received on and a timestamp
typedef struct {
    can_frame_t frame;
    uint8_t     bus_id;
    uint32_t    timestamp_us;  // Microseconds since boot (lower 32 bits)
} can_tagged_frame_t;

// Callback for received frames
typedef void (*can_rx_callback_t)(const can_tagged_frame_t *frame, void *user_ctx);

// Abstract CAN interface - all CAN controllers implement this
typedef struct can_interface {
    esp_err_t (*init)(struct can_interface *self);
    esp_err_t (*start)(struct can_interface *self);
    esp_err_t (*stop)(struct can_interface *self);
    esp_err_t (*send)(struct can_interface *self, const can_frame_t *frame);
    void (*set_rx_callback)(struct can_interface *self, can_rx_callback_t cb, void *user_ctx);

    uint8_t bus_id;        // Bus index (1-based, matching DashPilot convention)
    void   *ctx;           // Driver-specific context
} can_interface_t;
