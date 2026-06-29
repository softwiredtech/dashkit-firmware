#pragma once

#include "can_interface.h"
#include "esp_err.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t automation_manager_init(void);

void automation_manager_on_frame(const can_tagged_frame_t *frame);

// Route a BLE config opcode (not a CAN frame) to the automations.
void automation_manager_config(uint8_t opcode, uint16_t value);

#ifdef __cplusplus
}
#endif
