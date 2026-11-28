#include "can_filter.h"
#include "dbc/dbc_generated.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "can_filter";

// Entries per device: one 512-byte filter write holds at most 127.
#define CAN_FILTER_MAX_ENTRIES  128

typedef struct {
    uint8_t  bus_id;
    uint32_t addr;
} can_filter_entry_t;

// The per-frame checks run in the CAN receive tasks for every frame on both buses, so for standard ids on
// the two CAN buses they are one bit lookup with no lock:
//   s_used: the ids dbc/used_messages.txt lists (fixed).
//   a set:  which standard ids go on, and whether the used_messages.txt ids do too. There is one per device
//           and one for the union of all of them. The writer fills the copy readers are not using and then
//           switches its index, so a reader sees the old set or the new one.
// Anything else (a bus the firmware injects, an extended id) is rare and goes through the locked lists.
#define FILTER_BUSES  2
#define FILTER_WORDS  64   // 2048 standard ids / 32

typedef struct {
    uint32_t bits[FILTER_BUSES][FILTER_WORDS];
    bool     used;   // the used_messages.txt ids as well
} filter_set_t;

typedef struct {
    bool               open;      // a device is connected in this slot
    bool               has_list;  // it wrote a list; otherwise it gets the used_messages.txt ids
    size_t             count;
    can_filter_entry_t entries[CAN_FILTER_MAX_ENTRIES];
    filter_set_t       set[2];
    volatile uint8_t   idx;
} slot_t;

static slot_t            s_slots[CAN_FILTER_SLOTS];
static filter_set_t      s_union[2];
static volatile uint8_t  s_union_idx;
static SemaphoreHandle_t s_lock = NULL;
static uint32_t          s_used[FILTER_BUSES][FILTER_WORDS];
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

static void set_add_list(filter_set_t *w, const slot_t *s)
{
    if (!s->has_list) {
        w->used = true;
        return;
    }
    for (size_t i = 0; i < s->count; i++) {
        if (fast_path(s->entries[i].bus_id, s->entries[i].addr)) {
            bit_set(w->bits[s->entries[i].bus_id], s->entries[i].addr);
        }
    }
}

// Rebuild the set of `slot` and the union of every connected device. Caller holds s_lock.
static void rebuild(uint8_t slot)
{
    slot_t *s = &s_slots[slot];
    filter_set_t *w = &s->set[s->idx ^ 1];
    memset(w, 0, sizeof(*w));
    if (s->open) set_add_list(w, s);
    s->idx ^= 1;

    filter_set_t *u = &s_union[s_union_idx ^ 1];
    memset(u, 0, sizeof(*u));
    bool any = false;
    for (uint8_t i = 0; i < CAN_FILTER_SLOTS; i++) {
        if (!s_slots[i].open) continue;
        set_add_list(u, &s_slots[i]);
        any = true;
    }
    if (!any) u->used = true;   // no device: what the hardware filters used to accept
    s_union_idx ^= 1;
}

esp_err_t can_filter_init(void)
{
    build_used();
    if (s_lock) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    rebuild(0);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool can_filter_is_used(uint8_t bus_id, uint32_t addr)
{
    return fast_path(bus_id, addr) && bit_test(s_used[bus_id], addr);
}

static void set_open(uint8_t slot, bool open)
{
    if (!s_lock || slot >= CAN_FILTER_SLOTS) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_slots[slot].open = open;
    s_slots[slot].has_list = false;
    s_slots[slot].count = 0;
    rebuild(slot);
    xSemaphoreGive(s_lock);
}

void can_filter_open(uint8_t slot) { set_open(slot, true); }
void can_filter_close(uint8_t slot) { set_open(slot, false); }

esp_err_t can_filter_set(uint8_t slot, const uint8_t *data, size_t len)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;
    if (slot >= CAN_FILTER_SLOTS) return ESP_ERR_INVALID_ARG;

    // Parse into a static scratch list first (the BLE host task that calls this has a small stack; only that
    // task writes filters): a malformed write leaves the device's previous list in place.
    static can_filter_entry_t parsed[CAN_FILTER_MAX_ENTRIES];
    size_t new_count = 0;
    size_t offset = 0;
    while (offset < len) {
        if (offset + 2 > len) {
            ESP_LOGW(TAG, "Truncated header at offset %u", (unsigned)offset);
            return ESP_ERR_INVALID_ARG;
        }
        uint8_t bus_id   = data[offset++];
        uint8_t num_addr = data[offset++];

        if (offset + (size_t)num_addr * 4 > len) {
            ESP_LOGW(TAG, "Truncated address list for bus %u", bus_id);
            return ESP_ERR_INVALID_ARG;
        }

        for (uint8_t i = 0; i < num_addr; i++) {
            if (new_count >= CAN_FILTER_MAX_ENTRIES) {
                ESP_LOGW(TAG, "Filter table overflow (%d entries max)", CAN_FILTER_MAX_ENTRIES);
                return ESP_ERR_NO_MEM;
            }
            uint32_t addr = ((uint32_t)data[offset + 0]      ) |
                            ((uint32_t)data[offset + 1] <<  8) |
                            ((uint32_t)data[offset + 2] << 16) |
                            ((uint32_t)data[offset + 3] << 24);
            offset += 4;
            parsed[new_count].bus_id = bus_id;
            parsed[new_count].addr   = addr;
            new_count++;
        }
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    slot_t *s = &s_slots[slot];
    memcpy(s->entries, parsed, new_count * sizeof(parsed[0]));
    s->count = new_count;
    s->has_list = new_count > 0;
    s->open = true;
    rebuild(slot);
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "Filter for slot %u from %u-byte write: %u entries%s", (unsigned)slot,
             (unsigned)len, (unsigned)new_count, new_count == 0 ? " (used_messages.txt ids)" : "");
    for (size_t i = 0; i < new_count; i++) {
        ESP_LOGI(TAG, "  [%u] bus=%u addr=0x%03lX", (unsigned)i, parsed[i].bus_id, (unsigned long)parsed[i].addr);
    }
    return ESP_OK;
}

static bool set_has(const filter_set_t *f, uint8_t bus_id, uint32_t addr)
{
    return (f->used && bit_test(s_used[bus_id], addr)) || bit_test(f->bits[bus_id], addr);
}

// A non-standard frame: in a device's list as an exact entry? Caller holds s_lock.
static bool slot_lists(const slot_t *s, uint8_t bus_id, uint32_t addr)
{
    if (!s->open || !s->has_list) return false;
    for (size_t i = 0; i < s->count; i++) {
        if (s->entries[i].bus_id == bus_id && s->entries[i].addr == addr) return true;
    }
    return false;
}

// The controllers receive both buses in full, so a device without a list can't mean "everything": it gets the
// ids dbc/used_messages.txt lists for that bus, which is what the hardware filters used to accept.
bool can_filter_should_forward(uint8_t bus_id, uint32_t addr)
{
    build_used();
    if (!s_lock) return can_filter_is_used(bus_id, addr);
    if (fast_path(bus_id, addr)) return set_has(&s_union[s_union_idx], bus_id, addr);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool match = false;
    for (uint8_t i = 0; i < CAN_FILTER_SLOTS && !match; i++) match = slot_lists(&s_slots[i], bus_id, addr);
    xSemaphoreGive(s_lock);
    return match;
}

bool can_filter_wants(uint8_t slot, uint8_t bus_id, uint32_t addr)
{
    if (slot >= CAN_FILTER_SLOTS || !s_lock) return false;
    const slot_t *s = &s_slots[slot];
    if (fast_path(bus_id, addr)) return set_has(&s->set[s->idx], bus_id, addr);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool match = slot_lists(s, bus_id, addr);
    xSemaphoreGive(s_lock);
    return match;
}
