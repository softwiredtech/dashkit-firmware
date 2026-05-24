#pragma once

#include "can_interface.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the CAN monitor task. Creates a queue and a FreeRTOS task that
// observes selected incoming CAN frames (before the BLE filter) and logs
// counter changes to the serial console.
esp_err_t can_monitor_init(void);

// Called from the CAN driver RX callback for every incoming frame, before
// any BLE filtering. Cheap: filters by bus/ID and only enqueues matches.
void can_monitor_observe(const can_tagged_frame_t *frame);

#ifdef __cplusplus
}
#endif
