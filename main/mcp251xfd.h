#pragma once

#include "can_interface.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t pin_mosi;
    gpio_num_t pin_sclk;
    gpio_num_t pin_miso;
    gpio_num_t pin_cs;
    gpio_num_t pin_int;
    int        spi_clock_hz;
    uint32_t   osc_freq_hz;    // External oscillator frequency
    uint32_t   bitrate;        // Nominal bitrate (e.g. 500000)
    uint32_t   bitrate_data;   // Data bitrate for CAN FD (0 = same as nominal)
    uint8_t    bus_id;         // Bus index for tagging frames
} mcp251xfd_config_t;

// Create an MCP251xFD CAN interface. Caller owns the returned pointer.
can_interface_t *mcp251xfd_create(const mcp251xfd_config_t *config);

// Destroy and free resources
void mcp251xfd_destroy(can_interface_t *iface);
