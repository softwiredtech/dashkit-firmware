#pragma once

#include "can_interface.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the auto wiper-off module. Loads the persisted enabled flag.
esp_err_t wiper_off_init(void);

// Arm or disarm the automation. Persisted to NVS so it survives a reboot. Set
// over BLE by the Android app (VC_CMD_WIPER_OFF_ENABLE) and re-synced on connect.
void wiper_off_set_enabled(bool enabled);

// Called from the CAN RX callback for every incoming frame on bus 1.
// Tracks the SCCM counter and triggers wiper-off when DAS_wiperSpeed goes 15->0.
void wiper_off_observe(const can_tagged_frame_t *frame);

#ifdef __cplusplus
}
#endif
