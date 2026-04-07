#include "can_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "can_mgr";

#define RX_QUEUE_DEPTH  128

static can_interface_t *s_interfaces[CAN_MANAGER_MAX_INTERFACES];
static int              s_iface_count = 0;
static QueueHandle_t    s_rx_queue = NULL;

// Callback invoked by CAN drivers from their RX tasks
static void on_frame_received(const can_tagged_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (s_rx_queue) {
        // Drop frame if queue is full rather than blocking the driver
        xQueueSend(s_rx_queue, frame, 0);
    }
}

esp_err_t can_manager_init(void)
{
    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
    if (!s_rx_queue) {
        return ESP_ERR_NO_MEM;
    }
    s_iface_count = 0;
    ESP_LOGI(TAG, "CAN manager initialized (queue depth=%d)", RX_QUEUE_DEPTH);
    return ESP_OK;
}

esp_err_t can_manager_add_interface(can_interface_t *iface)
{
    if (s_iface_count >= CAN_MANAGER_MAX_INTERFACES) {
        return ESP_ERR_NO_MEM;
    }
    iface->set_rx_callback(iface, on_frame_received, NULL);
    s_interfaces[s_iface_count++] = iface;
    ESP_LOGI(TAG, "Added interface bus_id=%d (total: %d)", iface->bus_id, s_iface_count);
    return ESP_OK;
}

esp_err_t can_manager_start(void)
{
    for (int i = 0; i < s_iface_count; i++) {
        esp_err_t err = s_interfaces[i]->init(s_interfaces[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Init failed for bus %d: %s", s_interfaces[i]->bus_id, esp_err_to_name(err));
            return err;
        }
        err = s_interfaces[i]->start(s_interfaces[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Start failed for bus %d: %s", s_interfaces[i]->bus_id, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t can_manager_stop(void)
{
    for (int i = 0; i < s_iface_count; i++) {
        s_interfaces[i]->stop(s_interfaces[i]);
    }
    return ESP_OK;
}

esp_err_t can_manager_receive(can_tagged_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
    if (xQueueReceive(s_rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}
