#pragma once

#include "can_interface.h"
#include "esp_err.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize every registered automation: call its init(), watch each
// subscribed message, and arm a shared-timer tick for periodic automations.
esp_err_t automation_manager_init(void);

// Dispatch a received frame to every automation subscribed to its id. Called
// once from the CAN RX path (replaces the old hardcoded *_observe() calls).
void automation_manager_on_frame(const can_tagged_frame_t *frame);

// Route a BLE config opcode (not a CAN frame) to the automations. Each
// automation's on_config decides whether the opcode is theirs.
void automation_manager_config(uint8_t opcode, uint16_t value);

#ifdef __cplusplus
}
#endif
