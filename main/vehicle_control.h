#pragma once

#include "esp_err.h"
#include <stdint.h>

// High-level Tesla vehicle-control commands.
//
// The Android app writes a small [opcode][value] packet to the BLE control
// characteristic (CADA0004); the firmware bit-packs the corresponding Tesla
// DBC signal into a CAN frame and transmits it on the vehicle bus.
//
// Only "comfort" signals that do NOT carry a rolling counter / checksum are
// exposed here, so each command is a self-contained frame. `value` is the raw
// signal value (firmware clamps it to the signal's bit width). See the command
// table in vehicle_control.c for the exact DBC signal each opcode maps to.
typedef enum {
    // --- UI_vehicleControl (0x273) ---
    VC_CMD_LOCK              = 0x01,  // UI_lockRequest: 1=LOCK 2=UNLOCK 3=REMOTE_UNLOCK 4=REMOTE_LOCK
    VC_CMD_CLOSURE           = 0x02,  // UI_remoteClosureRequest: 1=REAR_TRUNK 2=FRONT_TRUNK(frunk)
    VC_CMD_HONK_HORN         = 0x03,  // UI_honkHorn: 1=honk
    VC_CMD_MIRROR_FOLD       = 0x04,  // UI_mirrorFoldRequest: 1=RETRACT(fold) 2=PRESENT(unfold)
    VC_CMD_MIRROR_HEAT       = 0x05,  // UI_mirrorHeatRequest: 1=on
    VC_CMD_SEAT_HEAT_FL      = 0x06,  // UI_frontLeftSeatHeatReq: 0..3 (OFF/L1/L2/L3)
    VC_CMD_SEAT_HEAT_FR      = 0x07,  // UI_frontRightSeatHeatReq: 0..3
    VC_CMD_SEAT_HEAT_RL      = 0x08,  // UI_rearLeftSeatHeatReq: 0..3
    VC_CMD_SEAT_HEAT_RR      = 0x09,  // UI_rearRightSeatHeatReq: 0..3
    VC_CMD_SEAT_HEAT_RC      = 0x0A,  // UI_rearCenterSeatHeatReq: 0..3
    VC_CMD_DISPLAY_BRIGHTNESS= 0x0B,  // UI_displayBrightnessLevel: raw = percent / 0.5 (0..200)
    VC_CMD_DOME_LIGHT        = 0x0C,  // UI_domeLightSwitch: 0=OFF 1=ON 2=AUTO

    // --- UI_chargeRequest (0x333) ---
    VC_CMD_CHARGE_PORT_OPEN  = 0x0D,  // UI_openChargePortDoorRequest: 1=open
    VC_CMD_CHARGE_PORT_CLOSE = 0x0E,  // UI_closeChargePortDoorRequest: 1=close

    // --- UI_vehicleControl2 (0x3B3) ---
    VC_CMD_GLOVEBOX             = 0x0F,  // UI_gloveboxRequest: 1=open (latch release)

    // --- UI_hvacRequest (0x2F3) ---
    VC_CMD_CLIMATE_POWER     = 0x30,  // UI_hvacReqUserPowerState: 0=OFF 1=ON 2=PRECONDITION
    VC_CMD_CLIMATE_TEMP_LEFT = 0x31,  // UI_hvacReqTempSetpointLeft: raw=(degC-15)/0.5 (0=LO,26=HI)
    VC_CMD_CLIMATE_TEMP_RIGHT= 0x32,  // UI_hvacReqTempSetpointRight: raw=(degC-15)/0.5
    VC_CMD_CLIMATE_DEFROST   = 0x33,  // UI_hvacReqManualDefogState: 0=NONE 1=DEFOG 2=DEFROST
    VC_CMD_CLIMATE_AC        = 0x34,  // UI_hvacReqACDisable: 0=AUTO 1=OFF 2=ON

    // --- Battery preheat (faked UI_tripPlanning 0x082 injection) ---
    VC_CMD_BATTERY_PREHEAT   = 0x35,  // 1=start faking preheat, 0=stop
} vehicle_control_opcode_t;

// Create the command queue and worker task. Call once at startup.
esp_err_t vehicle_control_init(void);

// Enqueue a control command for transmission. Non-blocking and safe to call
// from the BLE GATT write callback. Returns ESP_ERR_INVALID_ARG for an unknown
// opcode, ESP_ERR_INVALID_STATE if not initialized.
esp_err_t vehicle_control_submit(uint8_t opcode, uint16_t value);
