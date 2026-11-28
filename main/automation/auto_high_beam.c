#include "automation.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "touch_enable_auto_high_beam";

#define NVS_NAMESPACE       "auto_high_beam"
#define NVS_KEY_ENABLED     "en"
#define HOLD_US             1500000
#define MAX_GAP_US          500000  // Allow for interleaved switch-status mux pages.
#define CONTROL_MAX_AGE_MS  1000
#define LIGHTING_MSG        "VCFRONT_lightingJuniper"
#define HIGH_BEAM_DAS_AUTO  3
#define BODY_CONTROL_MSG    "DAS_bodyControls"
#define UI_CONTROL_MSG      "UI_vehicleControl"

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
static const dbc_message_t *s_lighting_msg;
static dbc_sig_t s_high_beam_state_sig;
static const dbc_message_t *s_body_control_msg;
static dbc_sig_t s_override_sig;
static const dbc_message_t *s_ui_control_msg;
static dbc_sig_t s_ui_enable_sig;

static void auto_high_beam_init(automation_t *self)
{
    (void)self;
    dbc_msg_t msg = dbc_msg("VCLEFT_switchStatus");
    s_button_sig = dbc_sig(msg, "VCLEFT_swcHighBeamButtonState");
    dbc_msg_t control = dbc_msg(BODY_CONTROL_MSG);
    s_override_sig = dbc_sig(control, "DAS_ahlbOverride");
    if (s_button_sig >= 0 && s_override_sig >= 0) {
        s_mux_sig = g_dbc_signals[s_button_sig].mux_sel_sig;
        if (s_mux_sig >= 0) {
            s_switch_msg = &g_dbc_messages[msg];
            s_body_control_msg = &g_dbc_messages[control];
        }
    }

    dbc_msg_t lighting = dbc_msg(LIGHTING_MSG);
    s_high_beam_state_sig = dbc_sig(lighting, "VCFRONT_highBeamControlState");
    if (s_high_beam_state_sig >= 0) {
        s_lighting_msg = &g_dbc_messages[lighting];
    }

    dbc_msg_t ui_control = dbc_msg(UI_CONTROL_MSG);
    s_ui_enable_sig = dbc_sig(ui_control, "UI_autoHighBeamEnabled");
    if (s_ui_enable_sig >= 0) {
        s_ui_control_msg = &g_dbc_messages[ui_control];
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t enabled = 0;
        if (nvs_get_u8(nvs, NVS_KEY_ENABLED, &enabled) == ESP_OK) {
            s_enabled = enabled != 0;
        }
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "auto high beam ready (%s)", s_enabled ? "enabled" : "disabled");
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

    if (!send) {
        return;
    }
    if (!s_lighting_msg || !can_msg_fresh(LIGHTING_MSG, CONTROL_MAX_AGE_MS) ||
        !can_msg_fresh(BODY_CONTROL_MSG, CONTROL_MAX_AGE_MS)) {
        ESP_LOGI(TAG, "hold detected, skipping: lighting or body controls unavailable");
        return;
    }
    can_frame_t control;
    if (can_frame_live(s_lighting_msg->default_bus, LIGHTING_MSG, &control) != ESP_OK ||
        control.dlc != s_lighting_msg->dlc || control.extended) {
        ESP_LOGI(TAG, "hold detected, skipping: invalid lighting frame");
        return;
    }
    if (dbc_unpack(control.data, s_high_beam_state_sig) == HIGH_BEAM_DAS_AUTO) {
        ESP_LOGI(TAG, "hold detected, skipping: already in DAS Auto");
        return;
    }

    uint8_t bus = s_body_control_msg->default_bus;
    if (can_frame_live(bus, BODY_CONTROL_MSG, &control) != ESP_OK ||
        control.dlc != s_body_control_msg->dlc || control.extended) {
        ESP_LOGI(TAG, "hold detected, skipping: invalid body controls frame");
        return;
    }

    esp_err_t err;
    const char *signal;
    if (dbc_unpack(control.data, s_override_sig) == 1) {
        signal = "DAS_ahlbOverride";
        err = dbc_counter_seed_from_bus(bus, BODY_CONTROL_MSG);
        if (err == ESP_OK) {
            dbc_pack(control.data, s_override_sig, 0);
            err = can_frame_send(bus, BODY_CONTROL_MSG, &control);
        }
    } else {
        if (!s_ui_control_msg || !can_msg_fresh(UI_CONTROL_MSG, CONTROL_MAX_AGE_MS)) {
            ESP_LOGI(TAG, "hold detected, skipping: UI vehicle control unavailable");
            return;
        }
        bus = s_ui_control_msg->default_bus;
        if (can_frame_live(bus, UI_CONTROL_MSG, &control) != ESP_OK ||
            control.dlc != s_ui_control_msg->dlc || control.extended) {
            ESP_LOGI(TAG, "hold detected, skipping: invalid UI vehicle control frame");
            return;
        }
        signal = "UI_autoHighBeamEnabled";
        dbc_pack(control.data, s_ui_enable_sig, 0);
        err = can_frame_send(bus, UI_CONTROL_MSG, &control);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "hold detected, failed to clear %s: %s", signal, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "hold detected, sent %s=0", signal);
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

    ESP_LOGW(TAG, "automation %s", enabled ? "enabled" : "disabled");
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
