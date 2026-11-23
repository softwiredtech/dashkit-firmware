// Host unit tests for the app CAN filter (main/can/can_filter.c): what is forwarded to the app, and which
// frames the firmware's own hooks run for, now that the controllers deliver every frame on both buses.
// Runs against the real generated manifest tables. See tools/test/run_can_host_test.sh.

#include "can_filter.h"
#include "dbc_generated.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            failures++;                                                   \
        }                                                                 \
    } while (0)

static bool in_manifest(int bus, uint32_t id)
{
    for (int i = 0; i < g_dbc_hw_filters[bus].count; i++)
        if (g_dbc_hw_filters[bus].ids[i] == id) return true;
    return false;
}

// Filter wire format: per bus [bus][count][id LE32]...
static size_t entry(uint8_t *p, uint8_t bus, const uint32_t *ids, uint8_t n)
{
    size_t o = 0;
    p[o++] = bus; p[o++] = n;
    for (uint8_t i = 0; i < n; i++)
        for (int b = 0; b < 4; b++) p[o++] = (uint8_t)(ids[i] >> (8 * b));
    return o;
}

static uint32_t not_listed(int bus)
{
    for (uint32_t id = 0x700; id < 0x7FF; id++) if (!in_manifest(bus, id)) return id;
    return 0x7FE;
}

static void before_init_and_with_no_list_the_manifest_ids_are_forwarded(void)
{
    // Not "everything", as before the controllers received the whole bus.
    CHECK(can_filter_should_forward(1, g_dbc_hw_filters[1].ids[0]));
    CHECK(!can_filter_should_forward(1, not_listed(1)));

    CHECK(can_filter_init() == ESP_OK);
    for (int bus = 0; bus < DBC_HW_FILTER_BUS_COUNT; bus++) {
        for (uint32_t id = 0; id < 2048; id++) {
            CHECK(can_filter_should_forward((uint8_t)bus, id) == in_manifest(bus, id));
            CHECK(can_filter_is_used((uint8_t)bus, id) == in_manifest(bus, id));
        }
    }
    // Other buses and ids beyond 11 bits are never the firmware's, and not forwarded by default.
    CHECK(!can_filter_is_used(2, g_dbc_hw_filters[1].ids[0]));
    CHECK(!can_filter_is_used(1, 0x12345));
    CHECK(!can_filter_should_forward(2, g_dbc_hw_filters[1].ids[0]));
    CHECK(!can_filter_should_forward(1, 0x12345));
}

static void an_app_list_replaces_the_default_but_not_the_firmwares_set(void)
{
    uint32_t used = g_dbc_hw_filters[1].ids[0], extra = not_listed(1);
    uint8_t buf[64];
    size_t n = entry(buf, 1, (uint32_t[]){ extra }, 1);
    n += entry(buf + n, 0, (uint32_t[]){ 0x129 }, 1);
    n += entry(buf + n, 2, (uint32_t[]){ 0x321, 0x18DAF110 }, 2);     // an injected bus, an extended id
    CHECK(can_filter_set(buf, n) == ESP_OK);

    CHECK(can_filter_should_forward(1, extra));            // any id, not only the manifest's
    CHECK(!can_filter_should_forward(0, extra));           // per bus
    CHECK(can_filter_should_forward(0, 0x129));
    CHECK(!can_filter_should_forward(1, used));            // a manifest id the app did not ask for
    CHECK(can_filter_is_used(1, used));                    // ...still feeds the firmware's hooks
    CHECK(!can_filter_is_used(1, extra));                  // and an app-only id never does
    CHECK(can_filter_should_forward(2, 0x321));            // the locked list covers what the bitmaps do not
    CHECK(can_filter_should_forward(2, 0x18DAF110));
    CHECK(!can_filter_should_forward(2, 0x322));
    CHECK(!can_filter_should_forward(3, 0x321));

    // A second list replaces the first completely.
    n = entry(buf, 1, (uint32_t[]){ used }, 1);
    CHECK(can_filter_set(buf, n) == ESP_OK);
    CHECK(can_filter_should_forward(1, used));
    CHECK(!can_filter_should_forward(1, extra));
    CHECK(!can_filter_should_forward(0, 0x129));
    CHECK(!can_filter_should_forward(2, 0x321));

    // Clearing it (the app disconnected) restores the default.
    CHECK(can_filter_set(NULL, 0) == ESP_OK);
    CHECK(can_filter_should_forward(1, used));
    CHECK(!can_filter_should_forward(1, extra));
}

static void a_malformed_list_changes_nothing(void)
{
    uint32_t used = g_dbc_hw_filters[1].ids[0], extra = not_listed(1);
    uint8_t buf[16];
    size_t n = entry(buf, 1, (uint32_t[]){ extra }, 1);
    CHECK(can_filter_set(buf, n) == ESP_OK);
    CHECK(can_filter_set(buf, n - 1) != ESP_OK);           // truncated address
    CHECK(can_filter_set(buf, 1) != ESP_OK);               // truncated header
    CHECK(can_filter_should_forward(1, extra));
    CHECK(!can_filter_should_forward(1, used));
    CHECK(can_filter_set(NULL, 0) == ESP_OK);
}

int main(void)
{
    before_init_and_with_no_list_the_manifest_ids_are_forwarded();
    an_app_list_replaces_the_default_but_not_the_firmwares_set();
    a_malformed_list_changes_nothing();
    if (failures) {
        printf("%d check(s) FAILED\n", failures);
        return 1;
    }
    printf("ALL TESTS PASSED\n");
    return 0;
}
