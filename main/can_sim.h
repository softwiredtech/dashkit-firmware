#pragma once

// Start the DashKit CAN simulator task. Emits the message set that DashPilot's
// filter accepts (see DashKitDataSource.kt) with slowly-varying demo values.
// Only meaningful when built with DASHKIT_SIM_MODE.
void can_sim_start(void);
