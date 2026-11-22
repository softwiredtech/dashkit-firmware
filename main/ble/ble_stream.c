#include "ble_stream.h"

#include <string.h>

size_t ble_stream_build(const can_tagged_frame_t *frames, int count, int *from, uint8_t *buf, size_t buf_size,
                        ble_stream_want_fn want, void *ctx)
{
    size_t pos = 1;
    uint8_t n = 0;
    for (; *from < count; (*from)++) {
        const can_tagged_frame_t *f = &frames[*from];
        if (want && !want(f, ctx)) continue;
        uint8_t len = f->frame.dlc > 8 ? 8 : f->frame.dlc;  // DashPilot expects classic CAN (max 8)
        // timestamp(4) + bus(1) + addr(4) + len(1) + data(n)
        if (pos + 10 + len > buf_size) break;
        uint32_t ts = f->timestamp_us, id = f->frame.id;
        buf[pos + 0] = (uint8_t)ts;
        buf[pos + 1] = (uint8_t)(ts >> 8);
        buf[pos + 2] = (uint8_t)(ts >> 16);
        buf[pos + 3] = (uint8_t)(ts >> 24);
        buf[pos + 4] = f->bus_id;
        buf[pos + 5] = (uint8_t)id;
        buf[pos + 6] = (uint8_t)(id >> 8);
        buf[pos + 7] = (uint8_t)(id >> 16);
        buf[pos + 8] = (uint8_t)(id >> 24);
        buf[pos + 9] = len;
        memcpy(&buf[pos + 10], f->frame.data, len);
        pos += 10 + len;
        n++;
    }
    buf[0] = n;
    return n ? pos : 0;
}

int ble_stream_pick_primary(const ble_stream_cand_t *c, int n)
{
    int best = -1;
    for (int pass = 0; pass < 2 && best < 0; pass++) {   // writable receivers first, then any receiver
        for (int i = 0; i < n; i++) {
            if (!c[i].receiver || (pass == 0 && !c[i].writable)) continue;
            if (best < 0 || c[i].sub_seq > c[best].sub_seq) best = i;
        }
    }
    return best;
}
