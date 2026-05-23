#pragma once

#include "can_interface.h"
#include "esp_err.h"

// DAS_bodyControls (ID 1001 = 0x3E9) lives on Tesla's vehicle bus (bus 1 in
// DashKit). Auto-wiper is "disabled" while DAS reports DAS_wiperSpeed == 15
// (INVALID). When DAS transitions from 15 -> 0 (OFF), the car is re-enabling
// the auto-wiper logic, and this module will start re-asserting 15 at a
// higher rate than DAS itself to keep the auto behavior suppressed.

#define DAS_BODY_CONTROLS_ID    1001
#define DAS_BODY_CONTROLS_BUS   1
#define DAS_BODY_CONTROLS_DLC   8

// Initialize state and spawn the injection task. Must be called after
// can_manager_start() so can_manager_send() has interfaces to dispatch on.
esp_err_t wiper_override_init(void);

// Hook called from the CAN RX path for every received frame (pre-filter).
void wiper_override_on_rx(const can_tagged_frame_t *frame);
