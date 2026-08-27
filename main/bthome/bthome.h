#pragma once

// BTHome v2 advertisement listener (Shelly BLU buttons).
//
// Passively scans for BTHome service-data advertisements and maps button
// events to vehicle-control commands (currently: single press -> glovebox).
// Scanning runs alongside peripheral advertising; it must be paused while the
// Tesla central initiates a connection (NimBLE cannot scan and initiate at
// the same time).

// Begin scanning. Call once the host is synced (from on_sync).
void bthome_scan_start(void);

// Temporarily stop scanning (before ble_gap_connect).
void bthome_scan_pause(void);

// Restart scanning after a pause, if it was started.
void bthome_scan_resume(void);
