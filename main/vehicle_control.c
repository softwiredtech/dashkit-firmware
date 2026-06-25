#include "vehicle_control.h"
#include "can_interface.h"
#include "can_manager.h"
#include "battery_preheat.h"
#include "three_finger.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "veh_ctrl";

#define VC_BUS  1

// Tesla CAN message IDs (standard 11-bit).
#define ID_UI_VEHICLE_CONTROL   0x273  // 627  UI_vehicleControl
#define ID_UI_VEHICLE_CONTROL2  0x3B3  // 947  UI_vehicleControl2
#define ID_UI_CHARGE_REQUEST    0x333  // 819  UI_chargeRequest

// How aggressively to push each command onto the bus.
#define VC_BURST_REPEATS   10
#define VC_BURST_GAP_MS    20

// Glovebox is delivered as a read-modify-write of the live UI_vehicleControl2 frame
#define VC_GLOVEBOX_BURST    5
#define VC_GLOVEBOX_GAP_MS   10

typedef struct {
    uint8_t  opcode;
    uint32_t can_id;
    uint8_t  bus;
    uint8_t  start_bit;  // DBC Intel (little-endian) start bit
    uint8_t  length;     // signal width in bits
    uint8_t  dlc;        // frame length to transmit
} vc_command_t;

// Each opcode maps to exactly one Tesla DBC signal. `value` from the BLE
// packet is the raw signal value, clamped to `length` bits before packing.
static const vc_command_t s_commands[] = {
    // UI_vehicleControl (0x273, 8 bytes)
    { VC_CMD_CLOSURE,            ID_UI_VEHICLE_CONTROL,  VC_BUS, 54, 2, 8 },
    { VC_CMD_MIRROR_FOLD,        ID_UI_VEHICLE_CONTROL,  VC_BUS, 24, 2, 8 },
    { VC_CMD_GLOVEBOX,           ID_UI_VEHICLE_CONTROL2, VC_BUS,  0, 1, 8 },
    { VC_CMD_CHARGE_PORT_OPEN,   ID_UI_CHARGE_REQUEST, VC_BUS, 0, 1, 4 },
    { VC_CMD_CHARGE_PORT_CLOSE,  ID_UI_CHARGE_REQUEST, VC_BUS, 1, 1, 4 },
};

#define VC_COMMAND_COUNT  (sizeof(s_commands) / sizeof(s_commands[0]))

typedef struct {
    uint8_t  opcode;
    uint16_t value;
} vc_request_t;

static QueueHandle_t s_queue = NULL;

// Pack `length` bits of `value` into `data` starting at `start_bit`, using DBC
// Intel (little-endian) bit order: bit `start_bit` receives the LSB of value.
static void pack(uint8_t *data, int start_bit, int length, uint32_t value)
{
    uint32_t mask = (length >= 32) ? ~0u : ((1u << length) - 1u);
    value &= mask;
    for (int i = 0; i < length; i++) {
        int bit_pos  = start_bit + i;
        int byte_idx = bit_pos / 8;
        int bit_idx  = bit_pos % 8;
        if ((value >> i) & 1u) {
            data[byte_idx] |= (uint8_t)(1u << bit_idx);
        } else {
            data[byte_idx] &= (uint8_t)~(1u << bit_idx);
        }
    }
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

static void send_command(const vc_command_t *cmd, uint16_t value)
{
    can_frame_t frame = { .id = cmd->can_id, .dlc = cmd->dlc };
    memset(frame.data, 0, sizeof(frame.data));
    pack(frame.data, cmd->start_bit, cmd->length, value);

    ESP_LOGI(TAG, "cmd 0x%02X -> id=0x%03lX bit=%u len=%u val=%u",
             cmd->opcode, (unsigned long)cmd->can_id,
             cmd->start_bit, cmd->length, value);

    for (int i = 0; i < VC_BURST_REPEATS; i++) {
        can_manager_send(cmd->bus, &frame);
        vTaskDelay(pdMS_TO_TICKS(VC_BURST_GAP_MS));
    }
}

// Read-modify-write: copy the most recent live frame for this command's id,
// change only its signal, and transmit once.
static bool rmw_send_once(const vc_command_t *cmd, uint16_t value)
{
    can_frame_t frame;
    if (can_manager_get_frame(cmd->bus, cmd->can_id, &frame) != ESP_OK) {
        return false;
    }
    pack(frame.data, cmd->start_bit, cmd->length, value);
    can_manager_send(cmd->bus, &frame);
    return true;
}

// Glovebox: emulate a momentary press as a clean 0->1->0 pulse. The request is
// edge-triggered in the car, so we must drive it back to 0 after asserting;
// leaving it latched at 1 blocks the next open (including from the
// infotainment) until the car happens to rebroadcast a 0.
static void send_glovebox(const vc_command_t *cmd, uint16_t value)
{
    bool ok = false;
    for (int i = 0; i < VC_GLOVEBOX_BURST; i++) {
        ok = rmw_send_once(cmd, value);
        if (!ok) break;
        vTaskDelay(pdMS_TO_TICKS(VC_GLOVEBOX_GAP_MS));
    }
    if (!ok) {
        ESP_LOGW(TAG, "glovebox: no live 0x%03lX frame yet, skipping",
                 (unsigned long)cmd->can_id);
        return;
    }
    // Release: drive the request back to 0 so the line isn't left asserted.
    for (int i = 0; i < VC_GLOVEBOX_BURST; i++) {
        if (!rmw_send_once(cmd, 0)) break;
        vTaskDelay(pdMS_TO_TICKS(VC_GLOVEBOX_GAP_MS));
    }
    ESP_LOGI(TAG, "glovebox pulse sent (val=%u then released)", value);
}

static void vehicle_control_task(void *arg)
{
    (void)arg;
    vc_request_t req;
    while (true) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) == pdTRUE) {
            if (req.opcode == VC_CMD_BATTERY_PREHEAT) {
                // Not a single-shot CAN frame: toggles continuous injection.
                battery_preheat_set(req.value != 0);
                continue;
            }
            if (req.opcode == VC_CMD_THREE_FINGER_ACTION) {
                // Not a CAN frame: binds the three-finger tap to an action.
                three_finger_set_action((uint8_t)req.value);
                continue;
            }
            const vc_command_t *cmd = find_command(req.opcode);
            if (!cmd) {
                ESP_LOGW(TAG, "unknown opcode 0x%02X", req.opcode);
            } else if (cmd->opcode == VC_CMD_GLOVEBOX) {
                send_glovebox(cmd, req.value);
            } else {
                send_command(cmd, req.value);
            }
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
    // Glovebox replays a live UI_vehicleControl2 frame; keep a snapshot of it.
    can_manager_watch_frame(VC_BUS, ID_UI_VEHICLE_CONTROL2);
    ESP_LOGI(TAG, "vehicle control initialized (%u commands)", (unsigned)VC_COMMAND_COUNT);
    return ESP_OK;
}

esp_err_t vehicle_control_submit(uint8_t opcode, uint16_t value)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (opcode != VC_CMD_BATTERY_PREHEAT &&
        opcode != VC_CMD_THREE_FINGER_ACTION &&
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
