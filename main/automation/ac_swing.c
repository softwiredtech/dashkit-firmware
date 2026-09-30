// AC swing: RMW of the car's live UI_ventPanelControlRequest mux-0 frame.
// Left vent = driver (no RHD handling). Injected frames don't come back on RX,
// so a change in the car's own frame while swinging is the user moving a vent.
// On/off is never persisted: swing only starts on an explicit toggle.

#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"


static const char *TAG = "ac_swing";

#define VENT_BUS      1
#define VENT_MSG      "UI_ventPanelControlRequest"
#define VENT_ID       0x253
#define VENT_MUX_POS  0
#define SIG_MUX       "UI_ventPanelMultiplex"

#define X_MIN_RAW     0
#define X_MAX_RAW     200
#define SWEEP_MS      8000   // full intensity, one direction
#define INJECT_MS     40     // car idles at 2Hz
#define BASE_STALE_US (5 * 1000 * 1000)
#define MAX_ON_US     (30LL * 60 * 1000 * 1000)

#define PRESENCE_MSG  "UI_vehicleControl2"
#define PRESENCE_SIG  "UI_userPresent"
#define PRESENCE_ID   0x3B3
#define HVAC_MSG      "UI_hvacRequest"
#define HVAC_POWER    "UI_hvacReqUserPowerState"
#define HVAC_ID       0x2F3
#define HVAC_POWER_ON 1

#define NVS_NAMESPACE     "ac_swing"
#define NVS_KEY_SIDE      "side"
#define NVS_KEY_INTENSITY "int"

enum { SIDE_DRIVER, SIDE_PASSENGER, SIDE_BOTH, SIDE_COUNT };
enum { INTENSITY_LOW, INTENSITY_MEDIUM, INTENSITY_FULL, INTENSITY_COUNT };

static const char *const VENT_SIGS[] = {
    "UI_ventPanelLeftPositionX", "UI_ventPanelLeftPositionY", "UI_ventPanelLeftLateralSplit",
    "UI_ventPanelRightPositionX", "UI_ventPanelRightPositionY", "UI_ventPanelRightLateralSplit",
};
#define VENT_SIG_COUNT (sizeof(VENT_SIGS) / sizeof(VENT_SIGS[0]))
#define LEFT_X  0
#define RIGHT_X 3

static const uint8_t LOW_STEPS[]    = { 50, 100, 150 };
#define LOW_DWELL_MS    4000
static const uint8_t MEDIUM_STEPS[] = { 20, 52, 84, 116, 148, 180 };
#define MEDIUM_DWELL_MS 2000

static volatile bool    s_on;
static volatile int64_t s_on_since_us;
static volatile uint8_t s_side = SIDE_DRIVER;
static volatile uint8_t s_intensity = INTENSITY_FULL;
static volatile bool    s_active;
static int64_t          s_start_us;

static portMUX_TYPE s_base_lock = portMUX_INITIALIZER_UNLOCKED;
static can_frame_t  s_base;
static bool         s_have_base;
static int64_t      s_base_us;
static bool         s_ready;
static dbc_sig_t    s_sig_mux;
static dbc_sig_t    s_vent_sigs[VENT_SIG_COUNT];

static void inject_task(void *arg);

static void save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(nvs, NVS_KEY_SIDE, s_side);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, NVS_KEY_INTENSITY, s_intensity);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist config: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static void load_config(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    uint8_t v = 0;
    if (nvs_get_u8(nvs, NVS_KEY_SIDE, &v) == ESP_OK && v < SIDE_COUNT) {
        s_side = v;
    }
    if (nvs_get_u8(nvs, NVS_KEY_INTENSITY, &v) == ESP_OK && v < INTENSITY_COUNT) {
        s_intensity = v;
    }
    nvs_close(nvs);
}

static void ac_swing_init(automation_t *self)
{
    (void)self;
    load_config();
    dbc_msg_t msg = dbc_msg(VENT_MSG);
    s_sig_mux = dbc_sig(msg, SIG_MUX);
    bool resolved = s_sig_mux != DBC_SIG_INVALID;
    for (size_t i = 0; i < VENT_SIG_COUNT; i++) {
        s_vent_sigs[i] = dbc_sig(msg, VENT_SIGS[i]);
        resolved = resolved && s_vent_sigs[i] != DBC_SIG_INVALID;
    }
    if (!resolved) {
        ESP_LOGE(TAG, "vent signals missing from the DBC, AC swing disabled");
        return;
    }
    s_ready = true;
    xTaskCreatePinnedToCore(inject_task, "ac_swing_inj", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "AC swing READY, side %u, intensity %u", s_side, s_intensity);
}

static void turn_off(const char *why)
{
    if (s_on) {
        s_on = false;
        ESP_LOGW(TAG, "OFF (%s)", why);
    }
}

static bool signal_is(const char *msg, const char *sig, int expected)
{
    double v;
    return can_get(VENT_BUS, msg, sig, &v, false) == ESP_OK && (int)v == expected;
}

static bool base_fresh(void)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_base_lock);
    bool fresh = s_have_base && now - s_base_us < BASE_STALE_US;
    portEXIT_CRITICAL(&s_base_lock);
    return fresh;
}

static void toggle(void)
{
    if (s_on) {
        turn_off("toggled");
        return;
    }
    const char *refuse = NULL;
    if (!s_ready) {
        refuse = "not ready";
    } else if (!signal_is(PRESENCE_MSG, PRESENCE_SIG, 1)) {
        refuse = "driver not present";
    } else if (!signal_is(HVAC_MSG, HVAC_POWER, HVAC_POWER_ON)) {
        refuse = "climate off";
    } else if (!base_fresh()) {
        refuse = "no fresh vent frame";
    }
    if (refuse) {
        ESP_LOGW(TAG, "toggle ignored: %s", refuse);
        return;
    }
    s_on_since_us = esp_timer_get_time();
    s_on = true;
    ESP_LOGW(TAG, "ON");
}

static void ac_swing_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;
    if (!s_ready || frame->bus_id != VENT_BUS) {
        return;
    }
    if (s_on && frame->frame.id == PRESENCE_ID && signal_is(PRESENCE_MSG, PRESENCE_SIG, 0)) {
        turn_off("driver left");
        return;
    }
    if (s_on && frame->frame.id == HVAC_ID && !signal_is(HVAC_MSG, HVAC_POWER, HVAC_POWER_ON)) {
        turn_off("climate off");
        return;
    }
    if (frame->frame.id != VENT_ID || dbc_unpack(frame->frame.data, s_sig_mux) != VENT_MUX_POS) {
        return;
    }
    int64_t now = esp_timer_get_time();
    bool moved = false;
    portENTER_CRITICAL(&s_base_lock);
    if (s_have_base && now - s_base_us < BASE_STALE_US) {
        for (size_t i = 0; i < VENT_SIG_COUNT && !moved; i++) {
            moved = dbc_unpack(s_base.data, s_vent_sigs[i]) != dbc_unpack(frame->frame.data, s_vent_sigs[i]);
        }
    }
    s_base = frame->frame;
    s_have_base = true;
    s_base_us = now;
    portEXIT_CRITICAL(&s_base_lock);

    if (moved && s_active) {
        turn_off("vent moved on the screen");
    }
}

static uint8_t stepped_x(const uint8_t *steps, int count, int dwell_ms, int64_t elapsed_ms)
{
    int cycle = 2 * count - 2;
    int i = (int)((elapsed_ms / dwell_ms) % cycle);
    return steps[i < count ? i : cycle - i];
}

static uint8_t swing_x(int64_t elapsed_us)
{
    int64_t ms = elapsed_us / 1000;
    switch (s_intensity) {
    case INTENSITY_LOW:
        return stepped_x(LOW_STEPS, sizeof(LOW_STEPS), LOW_DWELL_MS, ms);
    case INTENSITY_MEDIUM:
        return stepped_x(MEDIUM_STEPS, sizeof(MEDIUM_STEPS), MEDIUM_DWELL_MS, ms);
    default: {
        int64_t t = ms % (2 * SWEEP_MS);
        if (t >= SWEEP_MS) {
            t = 2 * SWEEP_MS - t;
        }
        return (uint8_t)(X_MIN_RAW + (X_MAX_RAW - X_MIN_RAW) * t / SWEEP_MS);
    }
    }
}

static void set_active(bool active, const char *why, int64_t now)
{
    if (active == s_active) {
        return;
    }
    if (active) {
        s_start_us = now;
    }
    s_active = active;
    ESP_LOGW(TAG, "%s swing (%s)", active ? "START" : "STOP", why);
}

static void inject_task(void *arg)
{
    (void)arg;
    while (true) {
        const char *why = NULL;
        can_frame_t f;
        int64_t now = esp_timer_get_time();

        if (s_on && now - s_on_since_us >= MAX_ON_US) {
            turn_off("max duration reached");
        }
        if (!s_on) {
            why = "off";
        } else {
            bool fresh = false;
            portENTER_CRITICAL(&s_base_lock);
            if (s_have_base) {
                f = s_base;
                fresh = (now - s_base_us) < BASE_STALE_US;
            }
            portEXIT_CRITICAL(&s_base_lock);
            if (!fresh) {
                why = "no fresh vent frame";
            }
        }

        if (why) {
            set_active(false, why, now);
        } else {
            set_active(true, "on, live vent frame", now);
            uint8_t x = swing_x(now - s_start_us);
            if (s_side != SIDE_PASSENGER) {
                dbc_pack(f.data, s_vent_sigs[LEFT_X], x);
            }
            if (s_side != SIDE_DRIVER) {
                dbc_pack(f.data, s_vent_sigs[RIGHT_X], x);
            }
            can_frame_send(VENT_BUS, VENT_MSG, &f);
        }
        vTaskDelay(pdMS_TO_TICKS(INJECT_MS));
    }
}

static void ac_swing_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode == VC_CMD_AC_SWING_TOGGLE) {
        toggle();
        return;
    }
    if (opcode == VC_CMD_AC_SWING_SIDE) {
        uint8_t side = value < SIDE_COUNT ? (uint8_t)value : SIDE_DRIVER;
        if (side == s_side) {
            return;
        }
        ESP_LOGW(TAG, "side %u -> %u", s_side, side);
        s_side = side;
    } else if (opcode == VC_CMD_AC_SWING_INTENSITY) {
        uint8_t intensity = value < INTENSITY_COUNT ? (uint8_t)value : INTENSITY_FULL;
        if (intensity == s_intensity) {
            return;
        }
        ESP_LOGW(TAG, "intensity %u -> %u", s_intensity, intensity);
        s_intensity = intensity;
    } else {
        return;
    }
    save_config();
}

automation_t ac_swing_automation = {
    .name           = "ac_swing",
    .tick_period_ms = 0,
    .init           = ac_swing_init,
    .on_frame       = ac_swing_on_frame,
    .on_config      = ac_swing_on_config,
};
