#pragma once

#include "esp_err.h"
#include "host/ble_gatt.h"

// OTA BLE Service UUID: CADA0100-CA00-B1E0-B0D6-C000AA0100A1
// Control Characteristic: CADA0101-... (Write)
// Data Characteristic:    CADA0102-... (Write No Response)
// Status Characteristic:  CADA0103-... (Notify)

// Returns the GATT service definition for the OTA service.
// Must be called before ble_gatts_count_cfg/ble_gatts_add_svcs.
const struct ble_gatt_svc_def *ble_ota_get_service_def(void);

// Initialize OTA module state. Call after BLE server init.
esp_err_t ble_ota_init(void);

// Abort the in-progress OTA transfer if this link started it. Called on BLE
// disconnect.
void ble_ota_on_disconnect(uint16_t conn_handle);

// Check if OTA transfer is currently in progress.
bool ble_ota_is_in_progress(void);

// The link driving the in-progress transfer, or BLE_HS_CONN_HANDLE_NONE.
uint16_t ble_ota_conn_handle(void);
