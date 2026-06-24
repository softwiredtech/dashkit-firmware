#pragma once

#include "can_interface.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Action bound to an infotainment three-finger tap. Values match the payload
// the Android app sends with VC_CMD_THREE_FINGER_ACTION.
typedef enum {
    THREE_FINGER_ACTION_NONE        = 0,  // unbound / disabled
    THREE_FINGER_ACTION_GLOVEBOX    = 1,  // open glovebox
    THREE_FINGER_ACTION_PREHEAT     = 2,  // toggle battery preheat
    THREE_FINGER_ACTION_MIRROR_FOLD = 3,  // toggle mirror fold/unfold
} three_finger_action_t;

esp_err_t three_finger_init(void);

void three_finger_set_action(uint8_t action);

void three_finger_observe(const can_tagged_frame_t *frame);

#ifdef __cplusplus
}
#endif
