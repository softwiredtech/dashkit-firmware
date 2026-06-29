#include "vehicle_control.h"
#include "dbc.h"
#include "automation_manager.h"
#include "ble_server.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "veh_ctrl";

#define VC_BUS  1

// Burst commands ride a freshly-built frame: the car edge-detects the request,
// so we just resend the value a handful of times.
#define VC_BURST_REPEATS   10
#define VC_BURST_GAP_MS    20

// RMW-pulse commands must ride the car's LIVE frame (an isolated frame is
// ignored), so we read-modify-write the cached frame, holding the value for the
// command's hold_ms, then drive it back to 0 so the line isn't left asserted.
#define VC_RMW_GAP_MS        20
#define VC_RMW_RELEASE_REPS   5

// Each opcode maps to exactly one Tesla DBC message + signal (by name). `value`
// from the BLE packet is the raw signal value; the DBC engine handles packing.
// rmw=false bursts a fresh frame; rmw=true holds the value on the live frame for
// hold_ms then releases to 0.
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
    { VC_CMD_GLOVEBOX,          "UI_vehicleControl2", "UI_gloveboxRequest",            true,  60   },
    { VC_CMD_CHARGE_PORT_OPEN,  "UI_chargeRequest",   "UI_openChargePortDoorRequest",  true,  1000 },
    { VC_CMD_CHARGE_PORT_CLOSE, "UI_chargeRequest",   "UI_closeChargePortDoorRequest", true,  1000 },
};
#define VC_COMMAND_COUNT  (sizeof(s_commands) / sizeof(s_commands[0]))

typedef struct {
    uint8_t  opcode;
    uint16_t value;
} vc_request_t;

static QueueHandle_t s_queue = NULL;

// Config opcodes are not CAN frames: they route to an automation's on_config.
static bool is_config_opcode(uint8_t opcode)
{
    return opcode == VC_CMD_BATTERY_PREHEAT ||
           opcode == VC_CMD_MULTI_FINGER_ACTION ||
           opcode == VC_CMD_WIPER_OFF_ENABLE;
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
// write, so unrelated bits in the frame are preserved), then drive it back to 0
static void send_rmw_pulse(const vc_command_t *cmd, uint16_t value)
{
    int hold_reps = cmd->hold_ms / VC_RMW_GAP_MS;
    if (hold_reps < 1) {
        hold_reps = 1;
    }

    bool ok = false;
    for (int i = 0; i < hold_reps; i++) {
        ok = (can_send_live(VC_BUS, cmd->msg, cmd->sig, value, false) == ESP_OK);
        if (!ok) break;
        vTaskDelay(pdMS_TO_TICKS(VC_RMW_GAP_MS));
    }
    if (!ok) {
        ESP_LOGW(TAG, "%s: no live %s frame yet, skipping", cmd->sig, cmd->msg);
        return;
    }
    for (int i = 0; i < VC_RMW_RELEASE_REPS; i++) {
        if (can_send_live(VC_BUS, cmd->msg, cmd->sig, 0, false) != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(VC_RMW_GAP_MS));
    }
    ESP_LOGI(TAG, "%s.%s pulse (val=%u, held %ums, released)",
             cmd->msg, cmd->sig, value, cmd->hold_ms);
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
    ESP_LOGI(TAG, "vehicle control initialized (%u commands)", (unsigned)VC_COMMAND_COUNT);
    return ESP_OK;
}

esp_err_t vehicle_control_submit(uint8_t opcode, uint16_t value)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (opcode != VC_CMD_ENTER_PAIRING &&
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
