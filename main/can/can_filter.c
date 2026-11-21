#include "can_filter.h"
#include "dbc/dbc_generated.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "can_filter";

// Maximum number of (bus, addr) entries the filter can hold.
#define CAN_FILTER_MAX_ENTRIES  256

typedef struct {
    uint8_t  bus_id;
    uint32_t addr;
} can_filter_entry_t;

static can_filter_entry_t s_entries[CAN_FILTER_MAX_ENTRIES];
static volatile size_t    s_count = 0;
static SemaphoreHandle_t  s_lock = NULL;

// The per-frame checks run in the CAN receive tasks for every frame on both buses, so for standard ids on
// the two CAN buses they are one bit lookup with no lock:
//   s_used: the ids dbc/used_messages.txt lists (fixed).
//   s_app:  the app's list. The writer fills the set readers are not using and then switches s_app_idx, so a
//           reader sees the old list or the new one.
// Anything else (a bus the firmware injects, an extended id) is rare and goes through the locked list.
#define FILTER_BUSES  2
#define FILTER_WORDS  64   // 2048 standard ids / 32
static uint32_t          s_used[FILTER_BUSES][FILTER_WORDS];
static uint32_t          s_app[2][FILTER_BUSES][FILTER_WORDS];
static volatile uint8_t  s_app_idx;
static bool              s_used_built;

static inline bool bit_test(const uint32_t *bits, uint32_t id) { return (bits[id >> 5] >> (id & 31)) & 1u; }
static inline void bit_set(uint32_t *bits, uint32_t id) { bits[id >> 5] |= 1u << (id & 31); }
static inline bool fast_path(uint8_t bus_id, uint32_t addr) { return bus_id < FILTER_BUSES && addr <= 0x7FF; }

static void build_used(void)
{
    if (s_used_built) return;
    for (int b = 0; b < FILTER_BUSES && b < DBC_HW_FILTER_BUS_COUNT; b++) {
        const dbc_hw_filter_set_t *fs = &g_dbc_hw_filters[b];
        for (uint16_t i = 0; i < fs->count; i++) {
            if (fs->ids[i] <= 0x7FF) bit_set(s_used[b], fs->ids[i]);
        }
    }
    s_used_built = true;
}

esp_err_t can_filter_init(void)
{
    build_used();
    if (s_lock) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    s_count = 0;
    return ESP_OK;
}

bool can_filter_is_used(uint8_t bus_id, uint32_t addr)
{
    return fast_path(bus_id, addr) && bit_test(s_used[bus_id], addr);
}

esp_err_t can_filter_set(const uint8_t *data, size_t len)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;

    // Parse directly into s_entries under the lock to avoid a large
    // stack-allocated scratch buffer (the BLE host task that calls this
    // has a relatively small stack). s_count is only advanced on success,
    // so readers see either the previous or the new table.
    xSemaphoreTake(s_lock, portMAX_DELAY);

    size_t new_count = 0;
    size_t offset = 0;
    while (offset < len) {
        if (offset + 2 > len) {
            xSemaphoreGive(s_lock);
            ESP_LOGW(TAG, "Truncated header at offset %u", (unsigned)offset);
            return ESP_ERR_INVALID_ARG;
        }
        uint8_t bus_id   = data[offset++];
        uint8_t num_addr = data[offset++];

        if (offset + (size_t)num_addr * 4 > len) {
            xSemaphoreGive(s_lock);
            ESP_LOGW(TAG, "Truncated address list for bus %u", bus_id);
            return ESP_ERR_INVALID_ARG;
        }

        for (uint8_t i = 0; i < num_addr; i++) {
            if (new_count >= CAN_FILTER_MAX_ENTRIES) {
                xSemaphoreGive(s_lock);
                ESP_LOGW(TAG, "Filter table overflow (%d entries max)",
                         CAN_FILTER_MAX_ENTRIES);
                return ESP_ERR_NO_MEM;
            }
            uint32_t addr = ((uint32_t)data[offset + 0]      ) |
                            ((uint32_t)data[offset + 1] <<  8) |
                            ((uint32_t)data[offset + 2] << 16) |
                            ((uint32_t)data[offset + 3] << 24);
            offset += 4;

            s_entries[new_count].bus_id = bus_id;
            s_entries[new_count].addr   = addr;
            new_count++;
        }
    }

    if (new_count == 0) {
        // Back to the used_messages.txt ids. The bitmaps stay as they are: a
        // reader that already saw the old count finishes on the old list.
        s_count = 0;
    } else {
        // Mirror the list into the set readers are not using, then switch to it.
        uint8_t next = s_app_idx ^ 1;
        memset(s_app[next], 0, sizeof(s_app[next]));
        for (size_t i = 0; i < new_count; i++) {
            if (fast_path(s_entries[i].bus_id, s_entries[i].addr)) {
                bit_set(s_app[next][s_entries[i].bus_id], s_entries[i].addr);
            }
        }
        s_app_idx = next;
        s_count = new_count;
    }
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "Filter updated from %u-byte write: %u entries%s",
             (unsigned)len, (unsigned)new_count,
             new_count == 0 ? " (used_messages.txt ids)" : "");
    for (size_t i = 0; i < new_count; i++) {
        // Safe to read s_entries here without the lock: only this task
        // (the BLE host task) ever writes to the table.
        ESP_LOGI(TAG, "  [%u] bus=%u addr=0x%03lX",
                 (unsigned)i, s_entries[i].bus_id,
                 (unsigned long)s_entries[i].addr);
    }
    return ESP_OK;
}

// The controllers receive both buses in full, so an empty filter can't mean "everything": it forwards the ids
// dbc/used_messages.txt lists for that bus, which is what the hardware filters used to accept.
bool can_filter_should_forward(uint8_t bus_id, uint32_t addr)
{
    build_used();
    if (!s_lock || s_count == 0) return can_filter_is_used(bus_id, addr);
    if (fast_path(bus_id, addr)) return bit_test(s_app[s_app_idx][bus_id], addr);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t count = s_count;
    bool match = false;
    for (size_t i = 0; i < count && !match; i++) {
        if (s_entries[i].bus_id == bus_id && s_entries[i].addr == addr) {
            match = true;
        }
    }
    xSemaphoreGive(s_lock);
    return match;
}
