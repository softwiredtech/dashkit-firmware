#include "can_manager.h"
#include "can_filter.h"
#include "automation_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <string.h>

static const char *TAG = "can_mgr";

// Sized to absorb a few hundred ms of CAN traffic if BLE briefly stalls.
// At ~335 fps (realistic Tesla rates) this is ~750ms of headroom.
#define RX_QUEUE_DEPTH  256

static can_interface_t *s_interfaces[CAN_MANAGER_MAX_INTERFACES];
static int              s_iface_count = 0;
static QueueHandle_t    s_rx_queue = NULL;

// Snapshot cache of the most recent frame for a small set of watched IDs.
// Lets a sender grab a real, fully-populated frame off the bus and re-transmit
// it with only one signal changed (read-modify-write), instead of fabricating
// a frame that zeroes every other signal sharing the message.
#define FRAME_CACHE_SIZE  16
typedef struct {
    bool        used;
    bool        valid;
    uint8_t     bus_id;
    uint32_t    id;
    can_frame_t frame;
} frame_cache_entry_t;
static frame_cache_entry_t s_frame_cache[FRAME_CACHE_SIZE];
static portMUX_TYPE        s_cache_mux = portMUX_INITIALIZER_UNLOCKED;

// Refresh the cache entry (if any) matching this frame's bus + id. Hot path:
// runs in the driver RX task for every frame, so keep it to a quick scan.
static void frame_cache_update(const can_tagged_frame_t *f)
{
    portENTER_CRITICAL(&s_cache_mux);
    for (int i = 0; i < FRAME_CACHE_SIZE; i++) {
        if (s_frame_cache[i].used &&
            s_frame_cache[i].bus_id == f->bus_id &&
            s_frame_cache[i].id == f->frame.id) {
            s_frame_cache[i].frame = f->frame;
            s_frame_cache[i].valid = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_cache_mux);
}

// Callback invoked by CAN drivers from their RX tasks
static void on_frame_received(const can_tagged_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (!s_rx_queue) return;
    // Refresh the RMW cache first so automation on_frame handlers (which read
    // signals via can_get) observe this frame's current contents.
    frame_cache_update(frame);
    // Automations run before the BLE filter so they see all raw traffic.
    automation_manager_on_frame(frame);
    // Drop non-matching frames at the source so the bridge task's batching
    // timeout reflects "no more *matching* frames" rather than "no more
    // frames at all". This avoids holding onto a matching frame while we
    // wait through a flood of unrelated traffic.
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) return;

    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    // Drop frame if queue is full rather than blocking the driver
    xQueueSend(s_rx_queue, &stamped, 0);
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

esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame)
{
    if (!frame) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < s_iface_count; i++) {
        if (s_interfaces[i]->bus_id == bus_id) {
            if (!s_interfaces[i]->send) return ESP_ERR_NOT_SUPPORTED;
            return s_interfaces[i]->send(s_interfaces[i], frame);
        }
    }
    ESP_LOGW(TAG, "send: no interface for bus_id=%d", bus_id);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t can_manager_inject(const can_tagged_frame_t *frame)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) {
        return ESP_OK;  // Silently dropped by filter
    }
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    if (xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t can_manager_watch_frame(uint8_t bus_id, uint32_t id)
{
    esp_err_t err = ESP_ERR_NO_MEM;
    portENTER_CRITICAL(&s_cache_mux);
    for (int i = 0; i < FRAME_CACHE_SIZE; i++) {
        // Already watching this id: nothing to do.
        if (s_frame_cache[i].used &&
            s_frame_cache[i].bus_id == bus_id && s_frame_cache[i].id == id) {
            err = ESP_OK;
            break;
        }
    }
    if (err != ESP_OK) {
        for (int i = 0; i < FRAME_CACHE_SIZE; i++) {
            if (!s_frame_cache[i].used) {
                s_frame_cache[i].used   = true;
                s_frame_cache[i].valid  = false;
                s_frame_cache[i].bus_id = bus_id;
                s_frame_cache[i].id     = id;
                err = ESP_OK;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&s_cache_mux);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "watching frame bus=%u id=0x%03lX", bus_id, (unsigned long)id);
    } else {
        ESP_LOGW(TAG, "frame cache full, cannot watch id=0x%03lX", (unsigned long)id);
    }
    return err;
}

esp_err_t can_manager_get_frame(uint8_t bus_id, uint32_t id, can_frame_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    portENTER_CRITICAL(&s_cache_mux);
    for (int i = 0; i < FRAME_CACHE_SIZE; i++) {
        if (s_frame_cache[i].used &&
            s_frame_cache[i].bus_id == bus_id && s_frame_cache[i].id == id) {
            if (s_frame_cache[i].valid) {
                *out = s_frame_cache[i].frame;
                err = ESP_OK;
            }
            break;
        }
    }
    portEXIT_CRITICAL(&s_cache_mux);
    return err;
}
