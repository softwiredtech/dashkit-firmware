// Synthetic CAN data generator for DashPilot end-to-end validation.
//
// When CONFIG_DASHKIT_SIM_CAN is enabled, this task fabricates a few Tesla
// vehicle-CAN frames (matching the DashPilot app's Tesla decoder) and feeds them
// through can_manager_inject(), so they travel the real CAN->BLE forward path
// (can_to_ble_task -> build_ble_packet -> ble_server_notify) and are decoded by
// the app like genuine data. This lets the app complete first-time pairing
// (which gates "Connected" on receiving a CAN frame) and show live data on a
// bench where no vehicle bus is attached.
//
// TEST AID ONLY: gated by CONFIG_DASHKIT_SIM_CAN (default n). Keep off for
// production/on-car.
#include "sim_can.h"

#include "can_interface.h"
#include "can_manager.h"
#include "ble_server.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "sim_can"

#if defined(CONFIG_DASHKIT_SIM_CAN)

// Decoder bus indices used by the app's TeslaDashKitMapper: 0 = chassis
// (DI_speed), 1 = vehicle/powertrain (BMS, odometer).
#define BUS_CHASSIS 0
#define BUS_VEHICLE 1
#define INTERVAL_MS 50

// DBC Intel/little-endian signal pack ('@1+'). `start` is the LSB bit position;
// multi-byte signals fill bytes LSB-first per CAN little-endian layout.
static void pack_signal(uint8_t *d, int start, int len, uint32_t raw)
{
    for (int i = 0; i < len; i++) {
        if ((raw >> i) & 1u) {
            int byte = (start + i) / 8;
            int bit  = (start + i) % 8;
            d[byte] |= (uint8_t)(1u << bit);
        }
    }
}

static void inject(uint8_t bus, uint32_t id, int dlc, const uint8_t *data)
{
    can_tagged_frame_t f = { 0 };
    f.frame.id   = id;
    f.frame.dlc  = dlc;
    f.bus_id     = bus;
    memcpy(f.frame.data, data, dlc);
    can_manager_inject(&f);
}

// BO_ 599 DI_speed : 8 (0x257) -- DI_uiSpeed 24|8@1+ (1,0) kph (255 = SNA), bus 0
static void fake_speed(float kph)
{
    uint8_t d[8] = { 0 };
    uint8_t raw  = (uint8_t)(kph + 0.5f);
    if (raw > 254) raw = 254;   // 255 is SNA
    pack_signal(d, 24, 8, raw);
    inject(BUS_CHASSIS, 0x257, 8, d);
}

// BO_ 950 DI_odometerStatus : 4  (0x3B6) -- DI_odometer 0|32@1+ (0.001) km
static void fake_odometer(float km)
{
    uint8_t d[4] = { 0 };
    pack_signal(d, 0, 32, (uint32_t)(km * 1000.0f));
    inject(BUS_VEHICLE, 0x3B6, 4, d);
}

// BO_ 818 BMS_bmbMinMax : 6 (0x332), mux m0 THERM -- BMS_thermistorTMax 16|8@1+
//      (0.5,-40), BMS_thermistorTMin 24|8@1+ (0.5,-40); mux byte 0 = 0 selects
//      the THERM branch the app's pack-temp tile reads.
static void fake_pack_temp(float tmin, float tmax)
{
    uint8_t d[6] = { 0 };   // dlc 6, byte 0 = 0 => THERM mux
    pack_signal(d, 16, 8, (uint32_t)((tmax + 40.0f) / 0.5f));
    pack_signal(d, 24, 8, (uint32_t)((tmin + 40.0f) / 0.5f));
    inject(BUS_VEHICLE, 0x332, 6, d);
}

// BO_ 850 BMS_energyStatus : 8  (0x352), mux m0 -- BMS_nominalFullPackEnergy
//      16|16@1+ (0.02) kWh, BMS_nominalEnergyRemaining 32|16@1+ (0.02) kWh.
//      CarState computes SOC from these (nominalEnergyRemaining / fullPackEnergy),
//      so the muted names tie straight to the app's Battery tile.
static void fake_soc(float soc_pct)
{
    uint8_t d[8] = { 0 };
    float full    = 75.0f;
    float nominal = full * soc_pct / 100.0f;
    pack_signal(d, 16, 16, (uint32_t)(full    / 0.02f));
    pack_signal(d, 32, 16, (uint32_t)(nominal / 0.02f));
    inject(BUS_VEHICLE, 0x352, 8, d);   // mux index byte (0|2) stays 0 => m0 branch
}

// BO_ 306 BMS_hvBusStatus : 8 (0x132) -- BMS_packVoltage 0|16@1+ (0.01) V,
//      BMS_packCurrent 16|15@1- (-0.1) A
static void fake_hv(float volts, float amps)
{
    uint8_t d[8] = { 0 };
    pack_signal(d, 0, 16, (uint32_t)(volts / 0.01f));
    int32_t cur = (int32_t)(amps / -0.1f);          // negative factor
    pack_signal(d, 16, 15, (uint32_t)cur & 0x7FFFu);
    inject(BUS_VEHICLE, 0x132, 8, d);
}

// BO_ 594 BMS_powerAvailable : 8 (0x252) -- BMS_maxDischargePower 16|16@1+
//      (0.01) kW, BMS_maxRegenPower 0|16@1+ (0.01) kW
static void fake_power(float discharge_kw, float regen_kw)
{
    uint8_t d[8] = { 0 };
    pack_signal(d, 0, 16, (uint32_t)(regen_kw     / 0.01f));
    pack_signal(d, 16, 16, (uint32_t)(discharge_kw / 0.01f));
    inject(BUS_VEHICLE, 0x252, 8, d);
}

// BO_ 755 UI_hvacRequest : 5 (0x2F3) -- UI_hvacReqTempSetpointLeft 0|5@1+
//      (0.5,15) degC [0=LO .. 26=HI]
static void fake_ac_temp(float degc)
{
    uint8_t d[5] = { 0 };
    uint16_t raw = (uint16_t)((degc - 15.0f) / 0.5f);
    if (raw > 26) raw = 26;
    pack_signal(d, 0, 5, raw);
    inject(BUS_VEHICLE, 0x2F3, 5, d);
}

static void sim_can_task(void *arg)
{
    (void)arg;
    uint32_t t = 0;
    ESP_LOGI(TAG, "CAN simulator active: synthetic Tesla frames while a client is connected");
    while (1) {
        if (ble_server_is_connected()) {
            // Seesaw speed 0..110 kph so the gauge visibly moves.
            float kph = (float)((t % 4400u) / 20u);          // 0..220
            if (kph > 110.0f) kph = 220.0f - kph;            // fold to max 110
            fake_speed(kph);
            fake_odometer(12345.0f + (float)(t / 100000u));  // odo ticks 1 km / ~100 s
            float soc = 60.0f + (float)((t % 2000u) / 100) - 10.0f; // ~50..70%
            fake_soc(soc);
            fake_hv(352.0f, -30.0f);
            fake_pack_temp(22.0f, 28.0f);
            fake_power(150.0f, 60.0f);
            fake_ac_temp(20.0f);
        }
        t += INTERVAL_MS;
        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));
    }
}

#endif // CONFIG_DASHKIT_SIM_CAN

void sim_can_start(void)
{
#if defined(CONFIG_DASHKIT_SIM_CAN)
    xTaskCreate(sim_can_task, "sim_can", 4096, NULL, 4, NULL);
#endif
}
