#pragma once

#include "esp_err.h"
#include <stdint.h>

// High-level Tesla vehicle-control commands.
//
// The Android app writes a small [opcode][value] packet to the BLE control
// characteristic (CADA0004); the firmware bit-packs the corresponding Tesla
// DBC signal into a CAN frame and transmits it on the vehicle bus.
typedef enum {
    // --- UI_vehicleControl (0x273) ---
    VC_CMD_CLOSURE           = 0x02,  // UI_remoteClosureRequest: 1=REAR_TRUNK 2=FRONT_TRUNK(frunk)
    VC_CMD_MIRROR_FOLD       = 0x04,  // UI_mirrorFoldRequest: 1=RETRACT(fold) 2=PRESENT(unfold)

    // --- UI_chargeRequest (0x333) ---
    VC_CMD_CHARGE_PORT_OPEN  = 0x0D,  // UI_openChargePortDoorRequest: 1=open
    VC_CMD_CHARGE_PORT_CLOSE = 0x0E,  // UI_closeChargePortDoorRequest: 1=close

    // --- UI_vehicleControl2 (0x3B3) ---
    VC_CMD_GLOVEBOX          = 0x0F,  // UI_gloveboxRequest: 1=open (latch release)

    // --- Battery preheat (faked UI_tripPlanning 0x082 injection) ---
    VC_CMD_BATTERY_PREHEAT   = 0x35,  // 1=start faking preheat, 0=stop

    // --- Infotainment multi-finger tap binding ---
    // Not a CAN frame: configures which action the firmware runs when it sees an
    // N-finger tap on UI_status2.UI_activeTouchPoints. The 16-bit value packs the
    // finger count (3..5) in the high byte and the action in the low byte:
    //   value = (fingers << 8) | multi_finger_action_t
    // action: 0=none 1=glovebox 2=preheat 3=mirror_fold 4=frunk 5=trunk
    //         6=charge_port.
    VC_CMD_MULTI_FINGER_ACTION = 0x40,

    // --- Auto wiper-off automation toggle ---
    // Not a CAN frame: arms/disarms the firmware's auto wiper-off automation.
    // value: 1=enable, 0=disable. Persisted in NVS by the wiper_off module.
    VC_CMD_WIPER_OFF_ENABLE = 0x41,

    // --- Enter BLE pairing mode ---
    // Not a CAN frame: opens a window during which one new (not-yet-bonded)
    // device may pair. Only an already-paired device can send this (the control
    // characteristic requires an encrypted link). value is ignored.
    VC_CMD_ENTER_PAIRING = 0x42,

    // --- Rear AC fan override toggle (UI_hvacRequest 0x2F3) ---
    VC_CMD_REAR_FAN_TOGGLE = 0x43,

    // --- Reboot the DashKit ---
    // Not a CAN frame: restarts the ESP32 (esp_restart). value is ignored.
    VC_CMD_REBOOT = 0x44,

    // --- Keepalive ping ---
    // Not a CAN frame: handled directly in the BLE layer (never queued). The app
    // writes it every ~15 s; once the first ping is seen, the firmware drops the
    // link after 60 s of silence so a dead app cannot squat on the connection
    // slot. value is ignored.
    VC_CMD_PING = 0x45,
} vehicle_control_opcode_t;

// Create the command queue and worker task. Call once at startup.
esp_err_t vehicle_control_init(void);

// Enqueue a control command for transmission. Non-blocking and safe to call
// from the BLE GATT write callback. Returns ESP_ERR_INVALID_ARG for an unknown
// opcode, ESP_ERR_INVALID_STATE if not initialized.
esp_err_t vehicle_control_submit(uint8_t opcode, uint16_t value);
