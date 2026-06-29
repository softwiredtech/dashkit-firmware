#pragma once

// Common interface every automation implements. The goal is "an automation is
// one file": a feature declares a single automation_t, lists the CAN messages it
// cares about by name, and the automation_manager drives it. No central
// dispatcher edits, no hardcoded message ids.
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

    // NULL-terminated list of DBC message names this automation observes. The
    // manager watches each (keeping the frame cache fresh) and only calls
    // on_frame for frames whose id is in this list. May be NULL (no frames).
    const char *const *subscribe;

    // If > 0, on_tick is invoked every this-many ms from the shared esp_timer
    // task (zero extra FreeRTOS tasks). on_tick MUST be non-blocking: no
    // vTaskDelay. can_send is fine (it only queues to the driver).
    uint32_t tick_period_ms;

    // Lifecycle / event hooks. All optional (may be NULL).
    void (*init)(struct automation *self);
    void (*on_frame)(struct automation *self, const can_tagged_frame_t *frame);
    void (*on_tick)(struct automation *self);
    void (*on_config)(struct automation *self, uint8_t opcode, uint16_t value);

    // Free-form per-automation state pointer (optional).
    void *state;
} automation_t;

#ifdef __cplusplus
}
#endif
