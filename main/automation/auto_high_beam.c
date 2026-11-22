#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "auto_high_beam";

#define NVS_NAMESPACE       "auto_high_beam"
#define NVS_KEY_ENABLED     "en"
#define HOLD_US             2000000
#define MAX_GAP_US          250000  // Allow for interleaved switch-status mux pages.
#define CONTROL_MAX_AGE_MS  1000
#define CONTROL_MSG         "UI_vehicleControl"

typedef enum {
    WAIT_FOR_INACTIVE,
    INACTIVE,
    HOLDING,
} hold_state_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_enabled;
static hold_state_t s_state;
static int64_t s_last_sample_us;
static int64_t s_hold_start_us;
static const dbc_message_t *s_switch_msg;
static dbc_sig_t s_button_sig;
static dbc_sig_t s_mux_sig;
static const dbc_message_t *s_control_msg;
static dbc_sig_t s_enable_sig;

static void auto_high_beam_init(automation_t *self)
{
    (void)self;
    dbc_msg_t msg = dbc_msg("VCLEFT_switchStatus");
    s_button_sig = dbc_sig(msg, "VCLEFT_swcHighBeamButtonState");
    dbc_msg_t control = dbc_msg(CONTROL_MSG);
    s_enable_sig = dbc_sig(control, "UI_autoHighBeamEnabled");
    if (s_button_sig >= 0 && s_enable_sig >= 0) {
        s_mux_sig = g_dbc_signals[s_button_sig].mux_sel_sig;
        if (s_mux_sig >= 0) {
            s_switch_msg = &g_dbc_messages[msg];
            s_control_msg = &g_dbc_messages[control];
        }
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t enabled = 0;
        if (nvs_get_u8(nvs, NVS_KEY_ENABLED, &enabled) == ESP_OK) {
            s_enabled = enabled != 0;
        }
        nvs_close(nvs);
    }
}

static void auto_high_beam_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;
    if (!s_switch_msg || frame->bus_id != s_switch_msg->default_bus ||
        frame->frame.id != s_switch_msg->id || frame->frame.extended ||
        frame->frame.dlc != s_switch_msg->dlc) {
        return;
    }
    const uint8_t *data = frame->frame.data;
    if (dbc_unpack(data, s_mux_sig) != g_dbc_signals[s_button_sig].mux_val) {
        return;
    }

    const int64_t button = dbc_unpack(data, s_button_sig);
    const int64_t now = esp_timer_get_time();
    bool send = false;
    portENTER_CRITICAL(&s_lock);
    if (s_enabled) {
        if (now - s_last_sample_us > MAX_GAP_US) {
            s_state = WAIT_FOR_INACTIVE;
        }
        s_last_sample_us = now;

        if (button == 0) {  // INACTIVE
            s_state = INACTIVE;
        } else if (button != 1) {  // HARD_PRESS or FAULT
            s_state = WAIT_FOR_INACTIVE;
        } else if (s_state == INACTIVE) {
            s_hold_start_us = now;
            s_state = HOLDING;
        } else if (s_state == HOLDING && now - s_hold_start_us >= HOLD_US) {
            s_state = WAIT_FOR_INACTIVE;
            send = true;
        }
    }
    portEXIT_CRITICAL(&s_lock);

    if (!send || !can_msg_fresh(CONTROL_MSG, CONTROL_MAX_AGE_MS)) {
        return;
    }
    uint8_t bus = s_control_msg->default_bus;
    can_frame_t control;
    if (can_frame_live(bus, CONTROL_MSG, &control) != ESP_OK ||
        control.dlc != s_control_msg->dlc || control.extended) {
        return;
    }

    dbc_pack(control.data, s_enable_sig, 0);
    esp_err_t err = can_frame_send(bus, CONTROL_MSG, &control);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send failed: %s", esp_err_to_name(err));
    }
}

static void auto_high_beam_on_config(automation_t *self, uint8_t opcode, uint16_t value)
{
    (void)self;
    if (opcode != VC_CMD_TOUCH_AUTO_HIGH_BEAM_ENABLE || value > 1) {
        return;
    }

    bool enabled = value != 0;
    portENTER_CRITICAL(&s_lock);
    bool changed = s_enabled != enabled;
    if (changed) {
        s_enabled = enabled;
        s_state = WAIT_FOR_INACTIVE;
    }
    portEXIT_CRITICAL(&s_lock);
    if (!changed) {
        return;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, NVS_KEY_ENABLED, enabled);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to persist enabled flag: %s", esp_err_to_name(err));
    }
}

automation_t auto_high_beam_automation = {
    .name      = "auto_high_beam",
    .init      = auto_high_beam_init,
    .on_frame  = auto_high_beam_on_frame,
    .on_config = auto_high_beam_on_config,
};
