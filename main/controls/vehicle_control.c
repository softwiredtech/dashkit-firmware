#include "vehicle_control.h"
#include "dbc.h"
#include "automation_manager.h"
#include "ble_server.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "veh_ctrl";

#define VC_BUS  1

#define VC_BURST_REPEATS   10
#define VC_BURST_GAP_MS    20

#define VC_RMW_GAP_MS        20
#define VC_RMW_RELEASE_REPS   5

#define REAR_FAN_AUTO       0
#define REAR_FAN_OFF        1
#define REAR_FAN_HIGH       4
#define REAR_FAN_INJECT_MS  10

typedef struct {
    uint8_t     opcode;
    const char *msg;
    const char *sig;
    bool        rmw;
    uint16_t    hold_ms;
} vc_command_t;

static const vc_command_t s_commands[] = {
    { VC_CMD_CLOSURE,           "UI_vehicleControl",  "UI_remoteClosureRequest",       false, 0    },
    { VC_CMD_MIRROR_FOLD,       "UI_vehicleControl",  "UI_mirrorFoldRequest",          false, 0    },
    { VC_CMD_GLOVEBOX,          "UI_vehicleControl2", "UI_gloveboxRequest",            true,  400  },
    { VC_CMD_CHARGE_PORT_OPEN,  "UI_chargeRequest",   "UI_openChargePortDoorRequest",  true,  1000 },
    { VC_CMD_CHARGE_PORT_CLOSE, "UI_chargeRequest",   "UI_closeChargePortDoorRequest", true,  1000 },
};
#define VC_COMMAND_COUNT  (sizeof(s_commands) / sizeof(s_commands[0]))

typedef struct {
    uint8_t  opcode;
    uint16_t value;
} vc_request_t;

static QueueHandle_t s_queue = NULL;

static volatile int s_rear_fan_target = -1;
static volatile int s_rear_fan_baseline = -1;

// Config opcodes are not CAN frames: they route to an automation's on_config.
static bool is_config_opcode(uint8_t opcode)
{
    return opcode == VC_CMD_BATTERY_PREHEAT ||
           opcode == VC_CMD_MULTI_FINGER_ACTION ||
           opcode == VC_CMD_WIPER_OFF_ENABLE ||
           opcode == VC_CMD_CLIMATE_KEEP_ENABLE;
}

static const vc_command_t *find_command(uint8_t opcode)
{
    for (size_t i = 0; i < VC_COMMAND_COUNT; i++) {
        if (s_commands[i].opcode == opcode) {
            return &s_commands[i];
        }
    }
    return NULL;
}

static void send_burst(const vc_command_t *cmd, uint16_t value)
{
    ESP_LOGI(TAG, "cmd 0x%02X -> %s.%s = %u", cmd->opcode, cmd->msg, cmd->sig, value);
    for (int i = 0; i < VC_BURST_REPEATS; i++) {
        can_send(VC_BUS, cmd->msg, cmd->sig, value, false);
        vTaskDelay(pdMS_TO_TICKS(VC_BURST_GAP_MS));
    }
}

// Hold the asserted value on the car's live frame for ~hold_ms (read-modify-
// write, so unrelated bits in the frame are preserved), then drive it back to 0.
// With no live frame seen yet (quiet bus), fall back to from-scratch frames.
static void send_rmw_pulse(const vc_command_t *cmd, uint16_t value)
{
    int hold_reps = cmd->hold_ms / VC_RMW_GAP_MS;
    if (hold_reps < 1) {
        hold_reps = 1;
    }

    can_frame_t probe;
    bool live = (can_frame_live(VC_BUS, cmd->msg, &probe) == ESP_OK);
    if (!live) {
        ESP_LOGW(TAG, "%s: no live %s frame, sending from scratch",
                 cmd->sig, cmd->msg);
    }

    for (int i = 0; i < hold_reps; i++) {
        esp_err_t err = live ? can_send_live(VC_BUS, cmd->msg, cmd->sig, value, false)
                             : can_send(VC_BUS, cmd->msg, cmd->sig, value, false);
        if (err != ESP_OK) return;
        vTaskDelay(pdMS_TO_TICKS(VC_RMW_GAP_MS));
    }
    for (int i = 0; i < VC_RMW_RELEASE_REPS; i++) {
        esp_err_t err = live ? can_send_live(VC_BUS, cmd->msg, cmd->sig, 0, false)
                             : can_send(VC_BUS, cmd->msg, cmd->sig, 0, false);
        if (err != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(VC_RMW_GAP_MS));
    }
    ESP_LOGI(TAG, "%s.%s pulse (val=%u, held %ums, released, live=%d)",
             cmd->msg, cmd->sig, value, cmd->hold_ms, live);
}

static void rear_fan_toggle(void)
{
    if (s_rear_fan_target >= 0) {
        ESP_LOGI(TAG, "rear fan: stop injecting (was %d)", s_rear_fan_target);
        s_rear_fan_target = -1;
        return;
    }
    double cur;
    int target;
    if (can_get(VC_BUS, "UI_hvacRequest", "UI_hvacReqSecondRowState", &cur, false) == ESP_OK) {
        int s = (int)cur;
        s_rear_fan_baseline = s;
        target = (s == REAR_FAN_AUTO || s == REAR_FAN_OFF) ? REAR_FAN_HIGH : REAR_FAN_OFF;
        ESP_LOGI(TAG, "rear fan: bus=%d, inject %d @ %dms", s, target, REAR_FAN_INJECT_MS);
    } else {
        s_rear_fan_baseline = -1;  // unknown: injector captures it on first read
        target = REAR_FAN_HIGH;  // no live frame yet: default to turning it on
        ESP_LOGW(TAG, "rear fan: no live frame, inject %d @ %dms", target, REAR_FAN_INJECT_MS);
    }
    s_rear_fan_target = target;
}

static bool rear_fan_externally_changed(void)
{
    double cur;
    if (can_get(VC_BUS, "UI_hvacRequest", "UI_hvacReqSecondRowState", &cur, false) != ESP_OK) {
        return false;
    }
    int v = (int)cur;
    if (s_rear_fan_baseline < 0) {
        s_rear_fan_baseline = v;
        return false;
    }
    return v != s_rear_fan_baseline;
}

static void rear_fan_inject_task(void *arg)
{
    (void)arg;
    while (true) {
        int target = s_rear_fan_target;
        if (target >= 0) {
            if (rear_fan_externally_changed()) {
                ESP_LOGI(TAG, "rear fan: bus changed from baseline %d, stop injecting",
                         s_rear_fan_baseline);
                s_rear_fan_target = -1;
            } else {
                can_send_live(VC_BUS, "UI_hvacRequest", "UI_hvacReqSecondRowState",
                              target, false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(REAR_FAN_INJECT_MS));
    }
}

static void vehicle_control_task(void *arg)
{
    (void)arg;
    vc_request_t req;
    while (true) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (req.opcode == VC_CMD_ENTER_PAIRING) {
            ble_server_enter_pairing_mode();
            continue;
        }
        if (req.opcode == VC_CMD_REAR_FAN_TOGGLE) {
            rear_fan_toggle();
            continue;
        }
        if (req.opcode == VC_CMD_REBOOT) {
            ESP_LOGW(TAG, "reboot requested, restarting");
            vTaskDelay(pdMS_TO_TICKS(200));  // let the BLE write ack flush
            esp_restart();
        }
        if (is_config_opcode(req.opcode)) {
            automation_manager_config(req.opcode, req.value);
            continue;
        }
        const vc_command_t *cmd = find_command(req.opcode);
        if (!cmd) {
            ESP_LOGW(TAG, "unknown opcode 0x%02X", req.opcode);
            continue;
        }
        if (cmd->rmw) {
            send_rmw_pulse(cmd, req.value);
        } else {
            send_burst(cmd, req.value);
        }
    }
}

esp_err_t vehicle_control_init(void)
{
    s_queue = xQueueCreate(8, sizeof(vc_request_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    xTaskCreatePinnedToCore(vehicle_control_task, "veh_ctrl", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(rear_fan_inject_task, "rear_fan_inj", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "vehicle control initialized (%u commands)", (unsigned)VC_COMMAND_COUNT);
    return ESP_OK;
}

esp_err_t vehicle_control_submit(uint8_t opcode, uint16_t value)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (opcode != VC_CMD_ENTER_PAIRING &&
        opcode != VC_CMD_REAR_FAN_TOGGLE &&
        opcode != VC_CMD_REBOOT &&
        !is_config_opcode(opcode) &&
        !find_command(opcode)) {
        ESP_LOGW(TAG, "reject unknown opcode 0x%02X", opcode);
        return ESP_ERR_INVALID_ARG;
    }
    vc_request_t req = { .opcode = opcode, .value = value };
    if (xQueueSend(s_queue, &req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "queue full, dropping cmd 0x%02X", opcode);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
