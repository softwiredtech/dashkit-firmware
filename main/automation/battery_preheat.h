#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the battery-preheat injector. Starts a disabled background task
// (no bus writes until enabled).
esp_err_t battery_preheat_init(void);

// Enable/disable the injection. While enabled, a background task transmits a
// fake UI_tripPlanning (0x082) frame at 10 Hz with the preheat fields forced
// on, faking a Supercharger destination so the car preconditions/heats the
// battery. Driven by the BLE VC_CMD_BATTERY_PREHEAT command.
void battery_preheat_set(bool enable);
bool battery_preheat_enabled(void);

#ifdef __cplusplus
}
#endif
