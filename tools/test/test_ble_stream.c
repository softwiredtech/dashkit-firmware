// Host unit tests for the CAN stream to several devices (main/ble/ble_stream.c): packet building with a
// per-device filter and MTU, and the choice of the primary device. See tools/test/run_can_host_test.sh.

#include "ble_stream.h"

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

static can_tagged_frame_t frame(uint8_t bus, uint32_t id, uint8_t dlc)
{
    can_tagged_frame_t f;
    memset(&f, 0, sizeof(f));
    f.bus_id = bus;
    f.frame.id = id;
    f.frame.dlc = dlc;
    for (uint8_t i = 0; i < dlc && i < 8; i++) f.frame.data[i] = (uint8_t)(id + i);
    f.timestamp_us = 0x01020304;
    return f;
}

static bool want_bus1(const can_tagged_frame_t *f, void *ctx)
{
    (void)ctx;
    return f->bus_id == 1;
}

static void packets_follow_the_device_filter_and_mtu(void)
{
    can_tagged_frame_t b[4] = { frame(1, 0x100, 8), frame(0, 0x200, 8), frame(1, 0x300, 2), frame(1, 0x400, 8) };
    uint8_t buf[512];
    int from = 0;

    // Everything fits: count, then 18 bytes per 8-byte frame and 12 for a 2-byte one.
    size_t len = ble_stream_build(b, 4, &from, buf, sizeof(buf), NULL, NULL);
    CHECK(from == 4 && buf[0] == 4 && len == 1 + 18 + 18 + 12 + 18);
    CHECK(buf[1] == 0x04 && buf[4] == 0x01 && buf[5] == 1 && buf[6] == 0x00 && buf[7] == 0x01);  // ts LE, bus, id LE
    CHECK(buf[10] == 8 && buf[11] == 0x00 && buf[12] == 0x01);                                  // len, data

    // The device's filter skips frames it did not ask for.
    from = 0;
    len = ble_stream_build(b, 4, &from, buf, sizeof(buf), want_bus1, NULL);
    CHECK(from == 4 && buf[0] == 3 && len == 1 + 18 + 12 + 18);

    // A small MTU splits the batch at frame boundaries, each packet with its own count.
    from = 0;
    len = ble_stream_build(b, 4, &from, buf, 1 + 18, want_bus1, NULL);
    CHECK(len == 19 && buf[0] == 1 && from == 2);   // the unwanted bus-0 frame is skipped on the way
    len = ble_stream_build(b, 4, &from, buf, 1 + 18, want_bus1, NULL);
    CHECK(len == 13 && buf[0] == 1 && from == 3);
    len = ble_stream_build(b, 4, &from, buf, 1 + 18, want_bus1, NULL);
    CHECK(len == 19 && buf[0] == 1 && from == 4);

    // A frame larger than the buffer makes no progress (the caller stops).
    from = 0;
    len = ble_stream_build(b, 4, &from, buf, 10, NULL, NULL);
    CHECK(len == 0 && from == 0);

    // Nothing wanted: no packet, and the batch is consumed.
    can_tagged_frame_t z[1] = { frame(0, 0x200, 8) };
    from = 0;
    len = ble_stream_build(z, 1, &from, buf, sizeof(buf), want_bus1, NULL);
    CHECK(len == 0 && from == 1);
}

static void primary_is_the_last_writable_subscriber(void)
{
    ble_stream_cand_t c[4] = {
        { true,  true,  5 },   // phone A
        { true,  false, 9 },   // head-up display, read-only, subscribed last
        { false, true,  12 },  // connected, not subscribed
        { true,  true,  7 },   // phone B
    };
    CHECK(ble_stream_pick_primary(c, 4) == 3);     // a read-only device never takes over
    c[3].writable = false;
    CHECK(ble_stream_pick_primary(c, 4) == 0);
    c[0].writable = false;
    CHECK(ble_stream_pick_primary(c, 4) == 1);     // only read-only devices: the latest still gets the stream first
    c[0].receiver = c[1].receiver = c[3].receiver = false;
    CHECK(ble_stream_pick_primary(c, 4) == -1);
}

int main(void)
{
    packets_follow_the_device_filter_and_mtu();
    primary_is_the_last_writable_subscriber();
    if (failures) {
        printf("%d check(s) FAILED\n", failures);
        return 1;
    }
    printf("ALL TESTS PASSED\n");
    return 0;
}
