// Host unit tests for the MCP251xFD receive/transmit core (main/can/mcp251xfd_stream.c).
//
// The driver core talks to a small MCP2518FD model over the same full-duplex
// SPI callback the firmware uses: registers, hardware filters, FIFO RAM, head /
// tail pointers, overflow and the transmit FIFO. Run with
// tools/test/run_can_host_test.sh (plain gcc, no ESP-IDF).

#include "mcp251xfd_stream.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            failures++;                                                   \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------------
// MCP2518FD model
// ---------------------------------------------------------------------------
#define NFIFO        32
#define REG_RXIF     0x020
#define REG_FIFOCON(n)  (0x05C + ((n) - 1) * 12)
#define REG_FIFOSTA(n)  (0x060 + ((n) - 1) * 12)
#define REG_FIFOUA(n)   (0x064 + ((n) - 1) * 12)
#define REG_FLTCON(n)   (0x1D0 + (n) * 4)
#define REG_FLTOBJ(n)   (0x1F0 + (n) * 8)
#define REG_MASK(n)     (0x1F4 + (n) * 8)
#define RAM          0x400

typedef struct {
    uint8_t reg[0x400];
    uint8_t ram[2048];
    int head[NFIFO], tail[NFIFO], count[NFIFO];
    bool ovf[NFIFO];

    // Frames "sent on the bus" by the TX FIFO.
    uint8_t txlog[64][16];
    int tx_count;
    bool hold_tx;            // TXREQ does not drain the TX FIFO (bus busy)

    // Fault injection.
    int base_shift;          // RAM layout differs from what the driver expects
    int glitch_head;         // next N FIFOSTA reads of a non-empty RX FIFO report head == tail
    int glitch_head_high;    // next N FIFOSTA reads of a non-full RX FIFO report a head 2 too far (erratum)
    int glitch_high_skip;    // ... after this many correct reads
    int fail_xfer;           // fail the Nth transfer from now (1 = next), 0 = never

    // Bookkeeping checked by the tests.
    int uinc_on_empty;       // driver released a message that was not there
    int xfers;               // transfers since the test last cleared it
    int max_xfer_len;
} chip_t;

static uint32_t reg32(const chip_t *c, int addr)
{
    return (uint32_t)c->reg[addr] | ((uint32_t)c->reg[addr + 1] << 8) |
           ((uint32_t)c->reg[addr + 2] << 16) | ((uint32_t)c->reg[addr + 3] << 24);
}
static int fifo_depth(const chip_t *c, int n) { return (int)((reg32(c, REG_FIFOCON(n)) >> 24) & 0x1F) + 1; }
static bool fifo_is_tx(const chip_t *c, int n) { return (reg32(c, REG_FIFOCON(n)) >> 7) & 1; }
static int fifo_base(const chip_t *c, int n)
{
    int base = c->base_shift;
    for (int i = 1; i < n; i++) base += fifo_depth(c, i) * 16;   // RAM follows the FIFO numbers
    return base;
}

// `consume`: this read is the one that uses up a pending head glitch (the SPI
// model reads a register byte by byte, so only one byte may consume it).
static uint32_t fifo_sta(chip_t *c, int n, bool consume)
{
    int depth = fifo_depth(c, n);
    uint32_t sta = 0;
    if (fifo_is_tx(c, n)) {
        if (c->count[n] < depth) sta |= 1u << 0;          // not full
        sta |= (uint32_t)c->tail[n] << 8;
    } else {
        if (c->count[n] > 0) sta |= 1u << 0;              // not empty
        if (c->count[n] >= (depth + 1) / 2) sta |= 1u << 1;
        if (c->count[n] == depth) sta |= 1u << 2;         // full
        if (c->ovf[n]) sta |= 1u << 3;
        int head = c->head[n];
        if (c->glitch_head > 0 && c->count[n] > 0 && c->count[n] < depth) {
            head = c->tail[n];                            // erratum: stale index
            if (consume) c->glitch_head--;
        } else if (c->glitch_head_high > 0 && c->count[n] > 0 && c->count[n] < depth - 2) {
            if (c->glitch_high_skip > 0) {
                if (consume) c->glitch_high_skip--;
            } else {
                head = (head + 2) % depth;                // erratum: index too large
                if (consume) c->glitch_head_high--;
            }
        }
        sta |= (uint32_t)head << 8;
    }
    return sta;
}

static uint32_t fifo_ua(const chip_t *c, int n)
{
    int idx = fifo_is_tx(c, n) ? c->head[n] : c->tail[n];
    return (uint32_t)(fifo_base(c, n) + idx * 16);
}

static uint32_t rxif(const chip_t *c)
{
    uint32_t v = 0;
    for (int n = 1; n < NFIFO; n++) {
        if (!fifo_is_tx(c, n) && c->count[n] > 0 && (reg32(c, REG_FIFOCON(n)) & 1)) v |= 1u << n;
    }
    return v;
}

static uint8_t chip_read(chip_t *c, int addr)
{
    if (addr >= RAM) return (addr - RAM) < 2048 ? c->ram[addr - RAM] : 0;
    if (addr >= REG_RXIF && addr < REG_RXIF + 4) return (uint8_t)(rxif(c) >> ((addr - REG_RXIF) * 8));
    for (int n = 1; n < NFIFO; n++) {
        if (addr >= REG_FIFOSTA(n) && addr < REG_FIFOSTA(n) + 4)
            return (uint8_t)(fifo_sta(c, n, addr == REG_FIFOSTA(n) + 1) >> ((addr - REG_FIFOSTA(n)) * 8));
        if (addr >= REG_FIFOUA(n) && addr < REG_FIFOUA(n) + 4)
            return (uint8_t)(fifo_ua(c, n) >> ((addr - REG_FIFOUA(n)) * 8));
    }
    return c->reg[addr];
}

static void chip_write32(chip_t *c, int addr, uint32_t v)
{
    for (int n = 1; n < NFIFO; n++) {
        if (addr == REG_FIFOCON(n)) {
            // UINC, TXREQ and FRESET act once and read back as 0.
            uint32_t keep = v & ~((1u << 8) | (1u << 9) | (1u << 10));
            for (int i = 0; i < 4; i++) c->reg[addr + i] = (uint8_t)(keep >> (i * 8));
            int depth = fifo_depth(c, n);
            if (v & (1u << 10)) { c->head[n] = c->tail[n] = c->count[n] = 0; c->ovf[n] = false; }
            if (v & (1u << 8)) {
                if (fifo_is_tx(c, n)) {
                    c->head[n] = (c->head[n] + 1) % depth;
                    c->count[n]++;
                } else if (c->count[n] > 0) {
                    c->tail[n] = (c->tail[n] + 1) % depth;
                    c->count[n]--;
                } else {
                    c->uinc_on_empty++;
                }
            }
            if ((v & (1u << 9)) && fifo_is_tx(c, n) && !c->hold_tx) {
                while (c->count[n] > 0) {
                    if (c->tx_count < 64)
                        memcpy(c->txlog[c->tx_count++], &c->ram[fifo_base(c, n) + c->tail[n] * 16], 16);
                    c->tail[n] = (c->tail[n] + 1) % depth;
                    c->count[n]--;
                }
            }
            return;
        }
        if (addr == REG_FIFOSTA(n)) {
            if (!(v & (1u << 3))) c->ovf[n] = false;      // only RXOVIF is clearable
            return;
        }
    }
    for (int i = 0; i < 4; i++) c->reg[addr + i] = (uint8_t)(v >> (i * 8));
}

static esp_err_t chip_xfer(void *io, const uint8_t *tx, uint8_t *rx, size_t len)
{
    chip_t *c = (chip_t *)io;
    c->xfers++;
    if ((int)len > c->max_xfer_len) c->max_xfer_len = (int)len;
    if (c->fail_xfer > 0 && --c->fail_xfer == 0) return ESP_FAIL;

    int op = tx[0] >> 4;
    int addr = ((tx[0] & 0x0F) << 8) | tx[1];
    size_t n = len - 2;
    if (op == 0x3) {                                       // READ
        if (rx) {
            rx[0] = rx[1] = 0;
            for (size_t i = 0; i < n; i++) rx[2 + i] = chip_read(c, addr + (int)i);
        }
    } else if (op == 0x2) {                                // WRITE
        if (addr >= RAM) {
            for (size_t i = 0; i < n; i++) c->ram[addr - RAM + i] = tx[2 + i];
        } else {
            CHECK(n == 4);                                 // the core only writes whole registers
            chip_write32(c, addr, (uint32_t)tx[2] | ((uint32_t)tx[3] << 8) |
                                  ((uint32_t)tx[4] << 16) | ((uint32_t)tx[5] << 24));
        }
    } else {
        CHECK(!"unexpected SPI opcode");
    }
    return ESP_OK;
}

// A frame arriving from the bus: stored through the lowest-numbered matching
// filter, or lost (with the overflow flag) when that FIFO is full. Returns the
// FIFO number, 0 if no filter matched, -1 on overflow.
static int chip_bus_rx(chip_t *c, uint32_t id, const uint8_t *data, int dlc)
{
    for (int f = 0; f < 32; f++) {
        uint8_t con = c->reg[REG_FLTCON(f / 4) + (f % 4)];
        if (!(con & 0x80)) continue;
        uint32_t obj = reg32(c, REG_FLTOBJ(f)) & 0x7FF, mask = reg32(c, REG_MASK(f)) & 0x7FF;
        if ((id & mask) != (obj & mask)) continue;
        int n = con & 0x1F;
        int depth = fifo_depth(c, n);
        if (c->count[n] == depth) { c->ovf[n] = true; return -1; }
        uint8_t *o = &c->ram[fifo_base(c, n) + c->head[n] * 16];
        memset(o, 0, 16);
        o[0] = (uint8_t)id; o[1] = (uint8_t)(id >> 8);
        o[4] = (uint8_t)dlc;
        memcpy(o + 8, data, (size_t)(dlc > 8 ? 8 : dlc));
        c->head[n] = (c->head[n] + 1) % depth;
        c->count[n]++;
        return n;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------
// An id that lands in lane `k`.
static uint32_t lane_id(int k, int n) { return 0x400u | ((uint32_t)k << MCPFD_LANE_SHIFT) | ((uint32_t)n & 0xF); }
// The lane (and its FIFO) 0x1F9 is routed to.
#define LANE_1F9   ((int)MCPFD_LANE_OF(0x1F9))
#define FIFO_1F9   MCPFD_LANE_FIFO(LANE_1F9)

typedef struct {
    chip_t chip;
    mcpfd_stream_t s;
    uint8_t reg_tx[MCPFD_REG_BUF_SIZE], reg_rx[MCPFD_REG_BUF_SIZE];
    uint8_t blk_tx[MCPFD_BLK_BUF_SIZE], blk_rx[MCPFD_BLK_BUF_SIZE];
    mcpfd_hooks_t hooks;

    int lock_depth;
    int xfers_at_lock, max_xfers_per_lock;
    int tx_every;                 // send a frame on every Nth unlock (0 = never)
    int unlocks;

    // Per-id sequence check: each injected frame carries its per-id counter.
    uint32_t sent_seq[2048], got_seq[2048];
    long delivered;
    uint32_t last_id;             // id of the last frame delivered
} rig_t;

static void rig_lock(void *ctx)
{
    rig_t *r = ctx;
    CHECK(r->lock_depth == 0);
    r->lock_depth++;
    r->xfers_at_lock = r->chip.xfers;
}

static void rig_unlock(void *ctx)
{
    rig_t *r = ctx;
    CHECK(r->lock_depth == 1);
    int used = r->chip.xfers - r->xfers_at_lock;
    if (used > r->max_xfers_per_lock) r->max_xfers_per_lock = used;
    r->lock_depth--;
    // A transmit squeezing in between two RX chunks, as the vehicle-control
    // task does on the device.
    if (r->tx_every && (++r->unlocks % r->tx_every) == 0) {
        can_frame_t f = { .id = 0x1F9, .dlc = 8, .data = { 0x60, 0, 0x10 } };
        r->lock_depth++;
        CHECK(mcpfd_stream_send(&r->s, &f) == ESP_OK);
        r->lock_depth--;
    }
}

static void rig_deliver(void *ctx, const can_frame_t *f)
{
    rig_t *r = ctx;
    CHECK(r->lock_depth == 0);                 // delivery never holds the SPI lock
    CHECK(f->id < 2048 && f->dlc == 8);
    uint32_t seq = (uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8) |
                   ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24);
    CHECK(seq == r->got_seq[f->id]);           // in order, no loss, no duplicate
    CHECK(f->data[7] == (uint8_t)(f->id ^ 0x5A));
    r->got_seq[f->id] = seq + 1;
    r->delivered++;
    r->last_id = f->id;
}

static void rig_init(rig_t *r)
{
    memset(r, 0, sizeof(*r));
    r->s.xfer = chip_xfer;
    r->s.io = &r->chip;
    r->s.reg_tx = r->reg_tx; r->s.reg_rx = r->reg_rx;
    r->s.blk_tx = r->blk_tx; r->s.blk_rx = r->blk_rx;
    r->hooks = (mcpfd_hooks_t){ rig_lock, rig_unlock, rig_deliver, r };
    mcpfd_stream_init(&r->s);
    CHECK(mcpfd_stream_config_fifos(&r->s) == ESP_OK);
    CHECK(mcpfd_stream_filters_program(&r->s) == ESP_OK);
    CHECK(mcpfd_stream_reset_rx(&r->s) == ESP_OK);
}

// Put one frame on the bus. Returns the FIFO it went to (-1 = overflow).
static int rig_inject(rig_t *r, uint32_t id)
{
    uint8_t d[8];
    uint32_t seq = r->sent_seq[id];
    d[0] = (uint8_t)seq; d[1] = (uint8_t)(seq >> 8); d[2] = (uint8_t)(seq >> 16); d[3] = (uint8_t)(seq >> 24);
    d[4] = d[5] = d[6] = 0;
    d[7] = (uint8_t)(id ^ 0x5A);
    int fifo = chip_bus_rx(&r->chip, id, d, 8);
    if (fifo > 0) r->sent_seq[id] = seq + 1;
    return fifo;
}

static int rig_step(rig_t *r)
{
    return mcpfd_stream_step(&r->s, &r->hooks);
}

static void rig_drain(rig_t *r)
{
    int guard = 100000;
    while (rig_step(r) > 0 && --guard) {}
    CHECK(guard > 0);
}

static void rig_check_all_delivered(rig_t *r)
{
    for (int id = 0; id < 2048; id++) CHECK(r->got_seq[id] == r->sent_seq[id]);
}

static void rig_check_clean(rig_t *r)
{
    const mcpfd_stats_t *st = &r->s.stats;
    CHECK(st->rx_head_glitch == 0 && st->rx_head_unconfirmed == 0 && st->rx_bad_status == 0);
    CHECK(st->rx_desync == 0);
    CHECK(st->spi_errors == 0);
    CHECK(r->chip.uinc_on_empty == 0);
}

static uint32_t rnd_state = 12345;
static uint32_t rnd(void) { rnd_state = rnd_state * 1664525u + 1013904223u; return rnd_state >> 8; }

// ---------------------------------------------------------------------------
// Driver core tests
// ---------------------------------------------------------------------------
static void layout_matches_the_chip(void)
{
    static rig_t r;
    rig_init(&r);
    // The core's precomputed RAM offsets are what the chip derives from the
    // FIFO sizes, and together they use the RAM exactly.
    for (int i = 0; i < MCPFD_LANES; i++) {
        CHECK(r.s.lane[i].fifo == MCPFD_LANE_FIFO(i));
        CHECK(fifo_base(&r.chip, MCPFD_LANE_FIFO(i)) == r.s.lane[i].base);
        CHECK(fifo_depth(&r.chip, MCPFD_LANE_FIFO(i)) == MCPFD_LANE_DEPTH);
        CHECK(!fifo_is_tx(&r.chip, MCPFD_LANE_FIFO(i)));
    }
    CHECK(fifo_base(&r.chip, MCPFD_FIFO_TX) == r.s.tx_base);
    const mcpfd_lane_t *last = &r.s.lane[MCPFD_LANES - 1];
    CHECK(last->base + last->depth * MCPFD_OBJ_SIZE == MCPFD_RAM_SIZE);
    CHECK(MCPFD_LANE_DEPTH <= 32);                       // the chip's largest FIFO
    CHECK(fifo_is_tx(&r.chip, MCPFD_FIFO_TX) && fifo_depth(&r.chip, MCPFD_FIFO_TX) == MCPFD_TX_DEPTH);
    CHECK(MCPFD_BLK_BUF_SIZE >= 2 + MCPFD_CHUNK * MCPFD_OBJ_SIZE && MCPFD_REG_BUF_SIZE >= 2 + MCPFD_OBJ_SIZE);
}

static void every_id_is_received_in_its_lane(void)
{
    static rig_t r;
    rig_init(&r);
    // No id list on the chip: every one of the 2048 ids is accepted, by two of its bits.
    for (uint32_t id = 0; id < 2048; id++) {
        CHECK(rig_inject(&r, id) == MCPFD_LANE_FIFO((int)MCPFD_LANE_OF(id)));
        rig_drain(&r);
    }
    CHECK(r.delivered == 2048);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
    // Four filters do it; the other 28 stay disabled.
    for (int f = MCPFD_LANES; f < 32; f++) CHECK(!(r.chip.reg[REG_FLTCON(f / 4) + (f % 4)] & 0x80));
}

static void accept_all_filter_for_the_self_test(void)
{
    static rig_t r;
    rig_init(&r);
    CHECK(mcpfd_stream_filters_accept_all_lane0(&r.s) == ESP_OK);
    CHECK(MCPFD_FIFO_RX0 == 1);                          // the self-test reads FIFO 1
    CHECK(rig_inject(&r, 0x123) == MCPFD_FIFO_RX0);
    CHECK(rig_inject(&r, 0x256) == MCPFD_FIFO_RX0);
    CHECK(rig_inject(&r, 0x7FF) == MCPFD_FIFO_RX0);
    // And the real routing replaces it completely.
    CHECK(mcpfd_stream_filters_program(&r.s) == ESP_OK);
    CHECK(mcpfd_stream_reset_rx(&r.s) == ESP_OK);
    CHECK(rig_inject(&r, lane_id(3, 0)) == MCPFD_LANE_FIFO(3));
}


static void long_random_stream_is_complete_and_ordered(void)
{
    static rig_t r;
    rig_init(&r);
    r.tx_every = 7;      // transmits interleave with the drain throughout
    long injected = 0, lost = 0;
    for (int round = 0; round < 60000; round++) {
        int burst = (int)(rnd() % 6);
        for (int i = 0; i < burst; i++) {
            uint32_t id = rnd() % 2048;
            int fifo = rig_inject(&r, id);
            if (fifo > 0) injected++; else lost++;
        }
        int steps = (int)(rnd() % 3);
        for (int i = 0; i < steps; i++) {
            int n = rig_step(&r);
            CHECK(n <= MCPFD_CHUNK);
        }
    }
    rig_drain(&r);
    CHECK(injected > 100000);
    CHECK(r.delivered == injected);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
    // The drain keeps up with these bursts: nothing is lost and no overflow is counted
    // (overflow_stays_on_the_flooded_lane covers the other case).
    CHECK(lost == 0);
    CHECK(r.s.stats.rx_overflow[0] + r.s.stats.rx_overflow[1] +
          r.s.stats.rx_overflow[2] + r.s.stats.rx_overflow[3] == 0);
    // One lock never covers more than: RXIF + status + overflow clear + the
    // confirming status read + chunk read + one release per message.
    CHECK(r.max_xfers_per_lock <= 5 + MCPFD_CHUNK);
    CHECK(r.chip.max_xfer_len <= 2 + MCPFD_CHUNK * MCPFD_OBJ_SIZE);
    CHECK(r.chip.tx_count > 0 && r.s.stats.tx_frames > 0);
}

static void a_frame_waits_for_at_most_one_chunk_per_other_lane(void)
{
    static rig_t r;
    rig_init(&r);
    // A deep backlog on three lanes, and one frame (a control-relevant id) alone in the fourth.
    for (int k = 0; k < MCPFD_LANES; k++) {
        if (k == LANE_1F9) continue;
        for (int i = 0; i < MCPFD_LANE_DEPTH; i++) CHECK(rig_inject(&r, lane_id(k, 0)) > 0);
    }
    CHECK(rig_inject(&r, 0x1F9) == FIFO_1F9);
    // It is delivered within one round: after at most one chunk of each other lane.
    int steps = 0;
    r.last_id = 0;
    while (r.last_id != 0x1F9 && steps < 100) { CHECK(rig_step(&r) > 0); steps++; }
    CHECK(steps <= MCPFD_LANES);
    CHECK(r.delivered <= (MCPFD_LANES - 1) * MCPFD_CHUNK + 1);
    rig_drain(&r);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
}


static void full_fifo_and_wraparound(void)
{
    static rig_t r;
    rig_init(&r);
    // Move the ring so the full FIFO wraps, then fill it exactly: head == tail
    // with the full flag set.
    for (int i = 0; i < 13; i++) rig_inject(&r, 0x1F9);
    rig_drain(&r);
    for (int i = 0; i < MCPFD_LANE_DEPTH; i++) CHECK(rig_inject(&r, 0x1F9) == FIFO_1F9);
    CHECK(r.chip.count[FIFO_1F9] == MCPFD_LANE_DEPTH);
    CHECK(rig_step(&r) == MCPFD_CHUNK);             // tail 13: full chunks up to the end of the ring
    CHECK(rig_step(&r) == MCPFD_CHUNK);
    CHECK(rig_step(&r) == MCPFD_LANE_DEPTH - 13 - 2 * MCPFD_CHUNK);   // the short run that ends the ring
    CHECK(rig_step(&r) == MCPFD_CHUNK);             // then full chunks from index 0
    rig_drain(&r);
    CHECK(r.delivered == 13 + MCPFD_LANE_DEPTH);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
    CHECK(r.s.stats.rx_overflow[LANE_1F9] == 0);
}


static void stale_head_index_costs_no_frames(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 5; i++) rig_inject(&r, 0x1F9);
    r.chip.glitch_head = 2;          // FIFOSTA reports head == tail though 5 frames wait
    rig_drain(&r);
    CHECK(r.s.stats.rx_head_glitch == 2);
    CHECK(r.delivered == 5);
    rig_check_all_delivered(&r);
    CHECK(r.chip.uinc_on_empty == 0);
}

// MCP2518FD erratum DS80000789E 6: the head index can read too large. Trusting
// it would deliver frames that are not there and release them.
static void too_large_head_index_is_not_trusted(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 3; i++) rig_inject(&r, 0x1F9);
    r.chip.glitch_head_high = 1;     // the first status read says 5 frames wait
    CHECK(rig_step(&r) == 3);        // the confirming read says 3
    CHECK(r.s.stats.rx_head_unconfirmed == 1);
    CHECK(r.chip.uinc_on_empty == 0 && r.chip.count[FIFO_1F9] == 0);
    CHECK(rig_step(&r) == 0);
    CHECK(r.delivered == 3);
    rig_check_all_delivered(&r);

    // A wrong value on the confirming read changes nothing: the smaller count wins.
    for (int i = 0; i < 3; i++) rig_inject(&r, 0x1F9);
    r.chip.glitch_high_skip = 1;
    r.chip.glitch_head_high = 1;
    CHECK(rig_step(&r) == 3);
    CHECK(r.chip.glitch_head_high == 0);                 // the wrong value was read
    CHECK(r.chip.uinc_on_empty == 0 && r.delivered == 6);
    rig_check_all_delivered(&r);
}

static void impossible_fifo_status_reads_nothing(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 30; i++) rig_inject(&r, lane_id(i % 3, i % 3));
    r.chip.base_shift = 8;           // the addresses the chip reports no longer line up with the core's table
    for (int i = 0; i < 10; i++) CHECK(rig_step(&r) == 0);   // 0: the receive task goes back to waiting
    CHECK(r.delivered == 0);
    CHECK(r.s.stats.rx_bad_status == 10);
    CHECK(r.chip.uinc_on_empty == 0 && r.chip.count[MCPFD_LANE_FIFO(0)] == 10);   // nothing released
    // Back in line (the read was a passing corruption): everything is still there.
    r.chip.base_shift = 0;
    rig_drain(&r);
    CHECK(r.delivered == 30);
    rig_check_all_delivered(&r);
}

static void spi_error_on_the_chunk_read_loses_nothing(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 6; i++) rig_inject(&r, 0x1F9);
    r.chip.fail_xfer = 4;            // RXIF, status, confirming status, then the RAM read fails
    CHECK(rig_step(&r) == 0);
    CHECK(r.s.stats.spi_errors == 1);
    rig_drain(&r);
    CHECK(r.delivered == 6);
    rig_check_all_delivered(&r);
    CHECK(r.chip.uinc_on_empty == 0);
}

static void spi_error_on_a_release_duplicates_nothing(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 6; i++) rig_inject(&r, 0x1F9);
    r.chip.fail_xfer = 6;            // RXIF, status, confirming status, RAM read, release, then the 2nd release fails
    CHECK(rig_step(&r) == 1);        // only the released frame counts as read
    CHECK(r.s.stats.spi_errors == 1 && r.delivered == 1);
    rig_drain(&r);                   // rig_deliver fails on any frame delivered twice
    CHECK(r.delivered == 6);
    rig_check_all_delivered(&r);
    CHECK(r.chip.uinc_on_empty == 0);
}

static void transmit_encodes_frames(void)
{
    static rig_t r;
    rig_init(&r);
    can_frame_t f = { .id = 0x1F9, .dlc = 8, .data = { 0x60, 0x00, 0x10, 0, 0, 0, 0, 0xAB } };
    CHECK(mcpfd_stream_send(&r.s, &f) == ESP_OK);
    CHECK(r.chip.tx_count == 1);
    const uint8_t *o = r.chip.txlog[0];
    CHECK(o[0] == 0xF9 && o[1] == 0x01 && o[2] == 0 && o[3] == 0);
    CHECK(o[4] == 8 && o[5] == 0);
    CHECK(memcmp(o + 8, f.data, 8) == 0);

    // Short frame: unused payload bytes are zero, not left over from before.
    can_frame_t g = { .id = 0x273, .dlc = 3, .data = { 1, 2, 3, 0xEE, 0xEE } };
    CHECK(mcpfd_stream_send(&r.s, &g) == ESP_OK);
    o = r.chip.txlog[1];
    CHECK(o[0] == 0x73 && o[1] == 0x02 && o[4] == 3);
    CHECK(o[8] == 1 && o[9] == 2 && o[10] == 3 && o[11] == 0 && o[12] == 0);

    // Extended id: SID = id[28:18], EID = id[17:0], IDE set.
    can_frame_t e = { .id = 0x18DAF110, .dlc = 1, .extended = true, .data = { 0x3E } };
    CHECK(mcpfd_stream_send(&r.s, &e) == ESP_OK);
    o = r.chip.txlog[2];
    uint32_t t0 = (uint32_t)o[0] | ((uint32_t)o[1] << 8) | ((uint32_t)o[2] << 16) | ((uint32_t)o[3] << 24);
    CHECK((t0 & 0x7FF) == (0x18DAF110u >> 18) && ((t0 >> 11) & 0x3FFFF) == (0x18DAF110u & 0x3FFFF));
    CHECK(o[4] == (1 | (1 << 4)));
    can_frame_t back;
    mcpfd_stream_parse(o, &back);
    CHECK(back.id == 0x18DAF110 && back.extended && back.dlc == 1 && back.data[0] == 0x3E);

    can_frame_t fd = { .id = 0x100, .dlc = 9 };
    CHECK(mcpfd_stream_send(&r.s, &fd) == ESP_ERR_NOT_SUPPORTED);
    CHECK(r.s.stats.tx_frames == 3 && r.s.stats.tx_full == 0);
}

static void transmit_reports_a_full_fifo(void)
{
    static rig_t r;
    rig_init(&r);
    r.chip.hold_tx = true;           // the bus is busy: nothing leaves the TX FIFO
    can_frame_t f = { .id = 0x1F9, .dlc = 8 };
    for (int i = 0; i < MCPFD_TX_DEPTH; i++) CHECK(mcpfd_stream_send(&r.s, &f) == ESP_OK);
    CHECK(mcpfd_stream_send(&r.s, &f) == ESP_ERR_NO_MEM);
    CHECK(r.s.stats.tx_full == 1 && r.s.stats.tx_frames == MCPFD_TX_DEPTH);
    CHECK(r.chip.count[MCPFD_FIFO_TX] == MCPFD_TX_DEPTH);   // the refused frame wrote nothing
    // Receive is unaffected by a stuck transmitter.
    rig_inject(&r, 0x1F9);
    rig_drain(&r);
    CHECK(r.delivered == 1);
}

static void idle_step_is_one_transfer(void)
{
    static rig_t r;
    rig_init(&r);
    r.chip.xfers = 0;
    CHECK(rig_step(&r) == 0);
    CHECK(r.chip.xfers == 1);        // just the RXIF read
    // A single waiting frame: RXIF, status, read, release.
    rig_inject(&r, 0x1F9);
    r.chip.xfers = 0;
    CHECK(rig_step(&r) == 1);
    CHECK(r.chip.xfers == 4);
    // A full chunk: RXIF, status, confirming status, read, and one release per frame.
    for (int i = 0; i < MCPFD_CHUNK; i++) rig_inject(&r, lane_id(0, 0));
    r.chip.xfers = 0;
    CHECK(rig_step(&r) == MCPFD_CHUNK);
    CHECK(r.chip.xfers == 4 + MCPFD_CHUNK);
}

static void lanes_take_turns(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 20; i++)
        for (int k = 0; k < MCPFD_LANES; k++) CHECK(rig_inject(&r, lane_id(k, 0)) == MCPFD_LANE_FIFO(k));
    // Four steps, four different lanes: no lane waits behind another's backlog.
    uint32_t before[MCPFD_LANES], seen = 0;
    for (int k = 0; k < MCPFD_LANES; k++) before[k] = r.s.stats.rx_frames[k];
    for (int i = 0; i < MCPFD_LANES; i++) CHECK(rig_step(&r) == MCPFD_CHUNK);
    for (int k = 0; k < MCPFD_LANES; k++) seen += (r.s.stats.rx_frames[k] - before[k]) == MCPFD_CHUNK;
    CHECK(seen == MCPFD_LANES);
    rig_drain(&r);
    rig_check_all_delivered(&r);
}

static void overflow_stays_on_the_flooded_lane(void)
{
    static rig_t r;
    rig_init(&r);
    int flooded = (LANE_1F9 + 2) % MCPFD_LANES, lost = 0;
    for (int i = 0; i < MCPFD_LANE_DEPTH + 9; i++) lost += rig_inject(&r, lane_id(flooded, 0)) < 0;
    CHECK(lost == 9);
    // A flood on one lane costs the other lanes nothing.
    for (int i = 0; i < 5; i++) CHECK(rig_inject(&r, 0x1F9) == FIFO_1F9);
    rig_drain(&r);
    for (int k = 0; k < MCPFD_LANES; k++) CHECK(r.s.stats.rx_overflow[k] == (k == flooded ? 1u : 0u));
    CHECK(!r.chip.ovf[MCPFD_LANE_FIFO(flooded)]);              // one event, flag cleared again
    rig_check_all_delivered(&r);                               // everything that was stored
    CHECK(r.got_seq[0x1F9] == 5);
}

// The interrupt-flag register the core polls is C1RXIF (0x020), not C1TXIF (0x024): with the wrong one a
// waiting frame is never seen.
static void polls_the_receive_interrupt_register(void)
{
    static rig_t r;
    rig_init(&r);
    rig_inject(&r, 0x1F9);
    CHECK(rig_step(&r) == 1);
}

int main(void)
{
    layout_matches_the_chip();
    every_id_is_received_in_its_lane();
    accept_all_filter_for_the_self_test();
    long_random_stream_is_complete_and_ordered();
    lanes_take_turns();
    overflow_stays_on_the_flooded_lane();
    polls_the_receive_interrupt_register();
    a_frame_waits_for_at_most_one_chunk_per_other_lane();
    full_fifo_and_wraparound();
    stale_head_index_costs_no_frames();
    too_large_head_index_is_not_trusted();
    impossible_fifo_status_reads_nothing();
    spi_error_on_the_chunk_read_loses_nothing();
    spi_error_on_a_release_duplicates_nothing();
    transmit_encodes_frames();
    transmit_reports_a_full_fifo();
    idle_step_is_one_transfer();

    if (failures) {
        printf("%d check(s) FAILED\n", failures);
        return 1;
    }
    printf("ALL TESTS PASSED\n");
    return 0;
}
