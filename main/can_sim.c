#include "can_sim.h"
#include "can_interface.h"
#include "can_manager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "can_sim";

// DashPilot's TeslaDashKitMapper reads different signals from each bus.
// Chassis bus 0: SCCM_steeringAngle, DI_systemStatus, DI_speed, DAS_status.
// Vehicle bus 1: VCLEFT/VCRIGHT_lightStatus, BMS_*, DI_odometerStatus.
#define BUS_CHASSIS  0
#define BUS_VEHICLE  1

// Base tick period; rate divisors below are expressed as tick counts.
#define SIM_TICK_MS  10

// Pack `length` bits of `value` into `data` starting at `start_bit`, using
// DBC Intel (little-endian) bit order. Bit `start_bit` receives the LSB of
// the value.
static void pack(uint8_t *data, int start_bit, int length, uint64_t value)
{
    uint64_t mask = (length >= 64) ? ~0ULL : ((1ULL << length) - 1);
    value &= mask;
    for (int i = 0; i < length; i++) {
        int bit_pos  = start_bit + i;
        int byte_idx = bit_pos / 8;
        int bit_idx  = bit_pos % 8;
        if ((value >> i) & 1ULL) {
            data[byte_idx] |= (uint8_t)(1u << bit_idx);
        } else {
            data[byte_idx] &= (uint8_t)~(1u << bit_idx);
        }
    }
}

// Sign-extend a signed value into the unsigned bit field of `length` bits.
static uint64_t pack_signed(int64_t value, int length)
{
    uint64_t mask = (length >= 64) ? ~0ULL : ((1ULL << length) - 1);
    return ((uint64_t)value) & mask;
}

static void inject(uint8_t bus_id, uint32_t addr, uint8_t dlc, const uint8_t *data)
{
    can_tagged_frame_t f = {0};
    f.bus_id     = bus_id;
    f.frame.id   = addr;
    f.frame.dlc  = dlc;
    memcpy(f.frame.data, data, dlc);
    can_manager_inject(&f);
}

// ---------------------------------------------------------------------------
// Per-message emitters
// ---------------------------------------------------------------------------

// 0x129 SCCM_steeringAngle - 100Hz
// SCCM_steeringAngle: start=16, len=14, scale=0.1, offset=-819.2
static void emit_steering(float angle_deg)
{
    uint8_t data[8] = {0};
    // raw = (phys - offset) / scale = (angle + 819.2) / 0.1
    int32_t raw = (int32_t)lroundf((angle_deg + 819.2f) / 0.1f);
    if (raw < 0) raw = 0;
    if (raw > 0x3FFF) raw = 0x3FFF;
    pack(data, 16, 14, (uint64_t)raw);
    inject(BUS_CHASSIS, 0x129, 8, data);
}

// 0x118 DI_systemStatus - 100Hz
// DI_gear at start=21, len=3
static void emit_di_system_status(uint8_t gear)
{
    uint8_t data[8] = {0};
    pack(data, 21, 3, gear & 0x07);
    inject(BUS_CHASSIS, 0x118, 8, data);
}

// 0x257 DI_speed - 50Hz
// DI_uiSpeed at start=24, len=8 (km/h or mph depending on units bit)
static void emit_di_speed(float speed)
{
    uint8_t data[8] = {0};
    int32_t raw = (int32_t)lroundf(speed);
    if (raw < 0) raw = 0;
    if (raw > 0xFF) raw = 0xFF;
    pack(data, 24, 8, (uint64_t)raw);
    inject(BUS_CHASSIS, 0x257, 8, data);
}

// 0x399 DAS_status - 25Hz
// DAS_blindSpotRearLeft: 4|2, DAS_blindSpotRearRight: 6|2,
// DAS_fusedSpeedLimit: 8|5 (scale 5),
// DAS_sideCollisionWarning: 32|2, DAS_laneDepartureWarning: 37|3
static void emit_das_status(uint8_t bs_left, uint8_t bs_right,
                            uint8_t speed_limit_kph, uint8_t side_warn,
                            uint8_t ldw)
{
    uint8_t data[8] = {0};
    pack(data,  4, 2, bs_left  & 0x03);
    pack(data,  6, 2, bs_right & 0x03);
    pack(data,  8, 5, (speed_limit_kph / 5) & 0x1F);
    pack(data, 32, 2, side_warn & 0x03);
    pack(data, 37, 3, ldw       & 0x07);
    inject(BUS_CHASSIS, 0x399, 8, data);
}

// 0x3E2 VCLEFT_lightStatus - 10Hz
// VCLEFT_turnSignalStatus at start=4, len=2
static void emit_vcleft_lights(uint8_t turn_signal)
{
    uint8_t data[8] = {0};
    pack(data, 4, 2, turn_signal & 0x03);
    inject(BUS_VEHICLE, 0x3E2, 8, data);
}

// 0x3E3 VCRIGHT_lightStatus - 10Hz
// VCRIGHT_turnSignalStatus at start=4, len=2
static void emit_vcright_lights(uint8_t turn_signal)
{
    uint8_t data[8] = {0};
    pack(data, 4, 2, turn_signal & 0x03);
    inject(BUS_VEHICLE, 0x3E3, 8, data);
}

// 0x352 BMS_energyStatus - 10Hz, multiplexed
// Mux at start=0, len=2
// m0: BMS_nominalFullPackEnergy 16|16 (0.02), BMS_nominalEnergyRemaining 32|16 (0.02)
// m1: BMS_energyBuffer 16|16 (0.01)
static void emit_bms_energy_status_m0(float full_kwh, float remaining_kwh)
{
    uint8_t data[8] = {0};
    pack(data, 0, 2, 0);
    pack(data, 16, 16, (uint64_t)lroundf(full_kwh / 0.02f));
    pack(data, 32, 16, (uint64_t)lroundf(remaining_kwh / 0.02f));
    inject(BUS_VEHICLE, 0x352, 8, data);
}

static void emit_bms_energy_status_m1(float buffer_kwh)
{
    uint8_t data[8] = {0};
    pack(data, 0, 2, 1);
    pack(data, 16, 16, (uint64_t)lroundf(buffer_kwh / 0.01f));
    inject(BUS_VEHICLE, 0x352, 8, data);
}

// 0x252 BMS_powerAvailable - 10Hz
// BMS_maxRegenPower 0|16 (0.01), BMS_maxDischargePower 16|16 (0.01)
static void emit_bms_power_available(float regen_kw, float discharge_kw)
{
    uint8_t data[8] = {0};
    pack(data,  0, 16, (uint64_t)lroundf(regen_kw     / 0.01f));
    pack(data, 16, 16, (uint64_t)lroundf(discharge_kw / 0.01f));
    inject(BUS_VEHICLE, 0x252, 8, data);
}

// 0x132 BMS_hvBusStatus - 10Hz
// BMS_packVoltage 0|16 (0.01), BMS_packCurrent 16|15 signed (-0.1)
static void emit_bms_hv_bus_status(float pack_voltage, float pack_current_a)
{
    uint8_t data[8] = {0};
    pack(data, 0, 16, (uint64_t)lroundf(pack_voltage / 0.01f));
    // Signed signal with scale -0.1: raw = phys / scale
    int32_t cur_raw = (int32_t)lroundf(pack_current_a / -0.1f);
    pack(data, 16, 15, pack_signed(cur_raw, 15));
    inject(BUS_VEHICLE, 0x132, 8, data);
}

// 0x332 BMS_bmbMinMax - 10Hz, mux 0 carries thermistor min/max
// Mux at start=0, len=2. Thermistors: scale 0.5, offset -40.
// Reasonable bit layout: TMin at 16|8, TMax at 24|8.
static void emit_bms_bmb_minmax(float t_min_c, float t_max_c)
{
    uint8_t data[8] = {0};
    pack(data, 0, 2, 0);
    int32_t tmin_raw = (int32_t)lroundf((t_min_c - (-40.0f)) / 0.5f);
    int32_t tmax_raw = (int32_t)lroundf((t_max_c - (-40.0f)) / 0.5f);
    if (tmin_raw < 0)    tmin_raw = 0;
    if (tmin_raw > 0xFF) tmin_raw = 0xFF;
    if (tmax_raw < 0)    tmax_raw = 0;
    if (tmax_raw > 0xFF) tmax_raw = 0xFF;
    pack(data, 16, 8, (uint64_t)tmin_raw);
    pack(data, 24, 8, (uint64_t)tmax_raw);
    inject(BUS_VEHICLE, 0x332, 8, data);
}

// 0x3B6 DI_odometerStatus - 10Hz
// DI_odometer 0|32 (0.001) km
static void emit_di_odometer(float odo_km)
{
    uint8_t data[8] = {0};
    pack(data, 0, 32, (uint64_t)lroundf(odo_km / 0.001f));
    inject(BUS_VEHICLE, 0x3B6, 8, data);
}

// ---------------------------------------------------------------------------
// Tick task
// ---------------------------------------------------------------------------

static void sim_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "CAN simulator running (chassis=bus %d, vehicle=bus %d)",
             BUS_CHASSIS, BUS_VEHICLE);

    uint32_t tick = 0;

    // Slowly varying state
    float    speed_kph      = 0.0f;
    float    speed_dir      = 1.0f;
    float    steering_deg   = 0.0f;
    float    steering_dir   = 1.0f;
    uint8_t  gear           = 4;        // 1=P 2=R 3=N 4=D (typical)
    uint8_t  left_blinker   = 0;
    uint8_t  right_blinker  = 0;
    float    odo_km         = 12345.678f;
    float    soc_kwh        = 55.0f;
    const float full_kwh    = 75.0f;
    float    buffer_kwh     = 3.5f;
    float    pack_voltage   = 380.0f;
    float    pack_current_a = 0.0f;
    float    t_min_c        = 22.0f;
    float    t_max_c        = 24.0f;
    uint8_t  bms_mux_toggle = 0;
    uint8_t  speed_limit_kph = 30;
    int8_t   speed_limit_dir = 1;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SIM_TICK_MS);

    while (true) {
        vTaskDelayUntil(&last_wake, period);
        tick++;

        // ---- Update state ----
        // Speed: ramps 0..120 km/h and back, ~2 km/h every 100ms (~6s sweep).
        if ((tick % 10) == 0) {
            speed_kph += speed_dir * 2.0f;
            if (speed_kph > 120.0f) { speed_kph = 120.0f; speed_dir = -1.0f; }
            if (speed_kph <   0.0f) { speed_kph =   0.0f; speed_dir =  1.0f; }

            // Drift odometer (km per 100ms) - 100x faster than realtime so
            // it's visibly ticking in the UI.
            odo_km += speed_kph / 36.0f;

            // SOC slowly drains while accelerating, regens while slowing
            float delta = -speed_kph * 0.0005f;
            if (speed_dir < 0) delta = fabsf(delta) * 0.4f;
            soc_kwh += delta;
            if (soc_kwh > full_kwh) soc_kwh = full_kwh;
            if (soc_kwh < 5.0f)     soc_kwh = 5.0f;

            // Pack current: positive on discharge, scales with speed
            pack_current_a = speed_kph * 0.8f * (speed_dir > 0 ? 1.0f : -0.3f);
            pack_voltage   = 380.0f - pack_current_a * 0.02f;

            // Thermals creep up under load
            t_max_c = 24.0f + speed_kph * 0.05f;
            t_min_c = t_max_c - 2.0f;
        }
        // Steering: ±180° sweep in ~6s, ~6° every 50ms.
        if ((tick % 5) == 0) {
            steering_deg += steering_dir * 6.0f;
            if (steering_deg >  180.0f) { steering_deg =  180.0f; steering_dir = -1.0f; }
            if (steering_deg < -180.0f) { steering_deg = -180.0f; steering_dir =  1.0f; }
        }
        // Blinker pattern: left-on, off, right-on, off — 500ms each phase.
        if ((tick % 50) == 0) {
            static uint8_t blink_state = 0;
            blink_state = (blink_state + 1) % 4;
            left_blinker  = (blink_state == 1) ? 1 : 0;
            right_blinker = (blink_state == 3) ? 1 : 0;
        }
        // Fused speed limit: cycle 30..120 km/h in 5 km/h steps, every 2s.
        if ((tick % 200) == 0) {
            int v = (int)speed_limit_kph + speed_limit_dir * 5;
            if (v > 120) { v = 120; speed_limit_dir = -1; }
            if (v <  30) { v =  30; speed_limit_dir =  1; }
            speed_limit_kph = (uint8_t)v;
        }

        // Rates match what a real Tesla puts on the wire so this sim is a
        // realistic load test for the BLE pipeline. Total ~335 fps.

        // ---- 100Hz: every tick ----
        emit_steering(steering_deg);
        emit_di_system_status(gear);

        // ---- 50Hz: every 2 ticks ----
        if ((tick % 2) == 0) {
            emit_di_speed(speed_kph);
        }

        // ---- 25Hz: every 4 ticks ----
        if ((tick % 4) == 0) {
            uint8_t bs_left  = (left_blinker  ? 1 : 0);
            uint8_t bs_right = (right_blinker ? 1 : 0);
            emit_das_status(bs_left, bs_right, speed_limit_kph, 0, 0);
        }

        // ---- 10Hz: every 10 ticks ----
        if ((tick % 10) == 0) {
            emit_vcleft_lights(left_blinker);
            emit_vcright_lights(right_blinker);

            // Alternate the two energy-status mux frames so both get sent
            if (bms_mux_toggle == 0) {
                emit_bms_energy_status_m0(full_kwh, soc_kwh);
            } else {
                emit_bms_energy_status_m1(buffer_kwh);
            }
            bms_mux_toggle ^= 1;

            emit_bms_power_available(50.0f, 150.0f);
            emit_bms_hv_bus_status(pack_voltage, pack_current_a);
            emit_bms_bmb_minmax(t_min_c, t_max_c);
            emit_di_odometer(odo_km);
        }
    }
}

void can_sim_start(void)
{
    xTaskCreatePinnedToCore(sim_task, "can_sim", 4096, NULL, 4, NULL, 0);
}
