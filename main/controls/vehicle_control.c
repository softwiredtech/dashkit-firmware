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

// How aggressively to push each one-shot command onto the bus.
#define VC_BURST_REPEATS   10
#define VC_BURST_GAP_MS    20

// Glovebox is delivered as a read-modify-write of the live UI_vehicleControl2 frame.
#define VC_GLOVEBOX_BURST    5
#define VC_GLOVEBOX_GAP_MS   10

// Charge port open/close: RMW-hold the request bit (matching the CID's observed
// pulse), then release so the line isn't left asserted.
#define VC_CHARGE_PORT_HOLD_MS   1000
#define VC_CHARGE_PORT_GAP_MS    25
#define VC_CHARGE_PORT_RELEASE   5

typedef enum {
    STYLE_BURST,     // zeroed frame, value packed, sent repeatedly
    STYLE_GLOVEBOX,  // RMW pulse of the live frame (assert then release)
    STYLE_CHARGE,    // RMW hold of the live frame (assert ~1s then release)
} vc_style_t;

// Each opcode maps to exactly one Tesla DBC message + signal (by name). `value`
// from the BLE packet is the raw signal value; the DBC engine handles packing.
typedef struct {
    uint8_t     opcode;
    const char *msg;
    const char *sig;
    vc_style_t  style;
} vc_command_t;

static const vc_command_t s_commands[] = {
    { VC_CMD_CLOSURE,           "UI_vehicleControl",  "UI_remoteClosureRequest",       STYLE_BURST    },
    { VC_CMD_MIRROR_FOLD,       "UI_vehicleControl",  "UI_mirrorFoldRequest",          STYLE_BURST    },
    { VC_CMD_GLOVEBOX,          "UI_vehicleControl2", "UI_gloveboxRequest",            STYLE_GLOVEBOX },
    { VC_CMD_CHARGE_PORT_OPEN,  "UI_chargeRequest",   "UI_openChargePortDoorRequest",  STYLE_CHARGE   },
    { VC_CMD_CHARGE_PORT_CLOSE, "UI_chargeRequest",   "UI_closeChargePortDoorRequest", STYLE_CHARGE   },
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

// Read-modify-write the live frame for this command's message: change only its
// signal and transmit once. Returns false if no live frame is cached yet.
static bool rmw_send_once(const vc_command_t *cmd, uint16_t value)
{
    return can_send_live(VC_BUS, cmd->msg, cmd->sig, value, false) == ESP_OK;
}

static void send_glovebox(const vc_command_t *cmd, uint16_t value)
{
    bool ok = false;
    for (int i = 0; i < VC_GLOVEBOX_BURST; i++) {
        ok = rmw_send_once(cmd, value);
        if (!ok) break;
        vTaskDelay(pdMS_TO_TICKS(VC_GLOVEBOX_GAP_MS));
    }
    if (!ok) {
        ESP_LOGW(TAG, "glovebox: no live %s frame yet, skipping", cmd->msg);
        return;
    }
    // Release: drive the request back to 0 so the line isn't left asserted.
    for (int i = 0; i < VC_GLOVEBOX_BURST; i++) {
        if (!rmw_send_once(cmd, 0)) break;
        vTaskDelay(pdMS_TO_TICKS(VC_GLOVEBOX_GAP_MS));
    }
    ESP_LOGI(TAG, "glovebox pulse sent (val=%u then released)", value);
}

static void send_charge_port(const vc_command_t *cmd)
{
    int reps = VC_CHARGE_PORT_HOLD_MS / VC_CHARGE_PORT_GAP_MS;
    bool ok = false;
    for (int i = 0; i < reps; i++) {
        ok = rmw_send_once(cmd, 1);
        if (!ok) break;
        vTaskDelay(pdMS_TO_TICKS(VC_CHARGE_PORT_GAP_MS));
    }
    if (!ok) {
        ESP_LOGW(TAG, "charge port: no live %s frame yet, skipping", cmd->msg);
        return;
    }
    // Release: drive the request bit back to 0 so it isn't left asserted.
    for (int i = 0; i < VC_CHARGE_PORT_RELEASE; i++) {
        if (!rmw_send_once(cmd, 0)) break;
        vTaskDelay(pdMS_TO_TICKS(VC_CHARGE_PORT_GAP_MS));
    }
    ESP_LOGI(TAG, "charge port %s pulse sent (held %d ms)",
             cmd->sig, VC_CHARGE_PORT_HOLD_MS);
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
            // Not a CAN frame: open a window for one new device to pair.
            ble_server_enter_pairing_mode();
            continue;
        }
        if (is_config_opcode(req.opcode)) {
            // Not a CAN frame: route to the owning automation's on_config.
            automation_manager_config(req.opcode, req.value);
            continue;
        }
        const vc_command_t *cmd = find_command(req.opcode);
        if (!cmd) {
            ESP_LOGW(TAG, "unknown opcode 0x%02X", req.opcode);
            continue;
        }
        switch (cmd->style) {
        case STYLE_GLOVEBOX:
            send_glovebox(cmd, req.value);
            break;
        case STYLE_CHARGE:
            send_charge_port(cmd);
            break;
        case STYLE_BURST:
        default:
            send_burst(cmd, req.value);
            break;
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
    // Glovebox and charge port replay live frames; the DBC value cache keeps
    // them automatically (can_send_live RMW base), so no watch needed.
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
