#pragma once

#include "sdkconfig.h"

// BTHome v2 advertisement listener (Shelly BLU buttons).
//
// Passively scans for BTHome service-data advertisements and maps button
// events to vehicle-control commands (currently: single press -> glovebox).
// Scanning runs alongside peripheral advertising; it must be paused while the
// Tesla central initiates a connection (NimBLE cannot scan and initiate at
// the same time).
//
// No-op stubs when CONFIG_DASHKIT_BTHOME is off, so call sites stay clean.

#if CONFIG_DASHKIT_BTHOME

// Begin scanning. Call once the host is synced (from on_sync).
void bthome_scan_start(void);

// Temporarily stop scanning (before ble_gap_connect).
void bthome_scan_pause(void);

// Restart scanning after a pause, if it was started.
void bthome_scan_resume(void);

#else

static inline void bthome_scan_start(void) {}
static inline void bthome_scan_pause(void) {}
static inline void bthome_scan_resume(void) {}

#endif
