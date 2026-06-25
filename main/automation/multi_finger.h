#pragma once

#include "can_interface.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Action bound to an infotainment multi-finger tap. Values match the
// gestureValue each control carries in the Android app (VehicleControls.kt) and
// are sent with VC_CMD_MULTI_FINGER_ACTION.
typedef enum {
    MULTI_FINGER_ACTION_NONE        = 0,  // unbound / disabled
    MULTI_FINGER_ACTION_GLOVEBOX    = 1,  // open glovebox
    MULTI_FINGER_ACTION_PREHEAT     = 2,  // toggle battery preheat
    MULTI_FINGER_ACTION_MIRROR_FOLD = 3,  // toggle mirror fold/unfold
    MULTI_FINGER_ACTION_FRUNK       = 4,  // open front trunk
    MULTI_FINGER_ACTION_TRUNK       = 5,  // open rear trunk
    MULTI_FINGER_ACTION_CHARGE_PORT = 6,  // toggle charge port open/close
} multi_finger_action_t;

// Lowest / highest finger count that can be bound to an action.
#define MULTI_FINGER_MIN_FINGERS  3
#define MULTI_FINGER_MAX_FINGERS  5

esp_err_t multi_finger_init(void);

// Bind `action` to a `fingers`-finger tap. `fingers` must be in
// [MULTI_FINGER_MIN_FINGERS, MULTI_FINGER_MAX_FINGERS]; other values are ignored.
void multi_finger_set_action(uint8_t fingers, uint8_t action);

void multi_finger_observe(const can_tagged_frame_t *frame);

#ifdef __cplusplus
}
#endif
