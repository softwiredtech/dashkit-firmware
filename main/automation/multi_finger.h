#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MULTI_FINGER_ACTION_NONE        = 0,  // unbound / disabled
    MULTI_FINGER_ACTION_GLOVEBOX    = 1,
    MULTI_FINGER_ACTION_PREHEAT     = 2,
    MULTI_FINGER_ACTION_MIRROR_FOLD = 3,
    MULTI_FINGER_ACTION_FRUNK       = 4,
    MULTI_FINGER_ACTION_TRUNK       = 5,
    MULTI_FINGER_ACTION_CHARGE_PORT = 6,
    MULTI_FINGER_ACTION_REAR_FAN    = 7,
} multi_finger_action_t;

#define MULTI_FINGER_MIN_FINGERS  3
#define MULTI_FINGER_MAX_FINGERS  5

#ifdef __cplusplus
}
#endif
