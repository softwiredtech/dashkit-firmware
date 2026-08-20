#pragma once

// TEST AID (CONFIG_DASHKIT_SIM_CAN): when enabled, a task injects synthetic
// Tesla CAN frames into the real CAN->BLE pipeline whenever a client is
// connected, so the DashPilot app can complete first-time pairing and display
// data without a live vehicle CAN bus. Disabled by default; keep off for
// production/on-car use.
void sim_can_start(void);
