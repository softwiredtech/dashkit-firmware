#pragma once

// Common interface every automation implements. The goal is "an automation is
// one file": a feature declares a single automation_t and the automation_manager
// drives it. No central dispatcher edits, no hardcoded message ids.
//
// The DBC engine keeps the latest frame of every generated message in its value
// cache (like opendbc's CANParser `vl`), so an automation can read any signal it
// wants at any time via can_get / can_frame_live. on_frame is just an ungated
// "a frame arrived" event; on_tick is a periodic poll. There is no subscribe
// list — read whatever you care about from the cache when called.
//
// To add an automation:
//   1. New .c file defining an automation_t (e.g. `automation_t foo_automation`).
//   2. One extern + one array entry in automation_registry.c.
//   3. One SRCS line in main/CMakeLists.txt.

#include "can_interface.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct automation {
    const char *name;

    // If > 0, on_tick is invoked every this-many ms from the shared esp_timer
    // task (zero extra FreeRTOS tasks). on_tick MUST be non-blocking: no
    // vTaskDelay. can_send is fine (it only queues to the driver).
    uint32_t tick_period_ms;

    // Lifecycle / event hooks. All optional (may be NULL).
    void (*init)(struct automation *self);
    // Called for EVERY received CAN frame (ungated). Read whatever cached
    // signals you care about via can_get; the triggering frame is also passed.
    void (*on_frame)(struct automation *self, const can_tagged_frame_t *frame);
    void (*on_tick)(struct automation *self);
    void (*on_config)(struct automation *self, uint8_t opcode, uint16_t value);

    // Free-form per-automation state pointer (optional).
    void *state;
} automation_t;

#ifdef __cplusplus
}
#endif
