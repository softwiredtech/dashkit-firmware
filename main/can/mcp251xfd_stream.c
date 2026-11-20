#include "mcp251xfd_stream.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Chip registers used here (see the MCP2518FD data sheet, DS20006027)
// ---------------------------------------------------------------------------
#define REG_C1RXIF          0x020   // 0x024 is C1TXIF
#define REG_C1FIFOCON(n)    (0x05C + ((n) - 1) * 12)
#define REG_C1FIFOSTA(n)    (0x060 + ((n) - 1) * 12)
#define REG_C1FLTCON(n)     (0x1D0 + (n) * 4)
#define REG_C1FLTOBJ(n)     (0x1F0 + (n) * 8)
#define REG_C1MASK(n)       (0x1F4 + (n) * 8)
#define RAM_BASE            0x400

#define SPI_CMD_READ(addr)  (0x3000 | ((addr) & 0xFFF))
#define SPI_CMD_WRITE(addr) (0x2000 | ((addr) & 0xFFF))

#define FIFOCON_TFNRFNIE    (1u << 0)    // RX: not-empty interrupt enable
#define FIFOCON_TXEN        (1u << 7)
#define FIFOCON_UINC        (1u << 8)
#define FIFOCON_TXREQ       (1u << 9)
#define FIFOCON_FRESET      (1u << 10)
#define FIFOCON_FSIZE_SHIFT 24

#define FIFOSTA_TFNRFNIF    (1u << 0)    // RX: not empty / TX: not full
#define FIFOSTA_TFERFFIF    (1u << 2)    // RX: full
#define FIFOSTA_RXOVIF      (1u << 3)    // RX: a frame was dropped (sticky)
#define FIFOSTA_FIFOCI(sta) (((sta) >> 8) & 0x1F)   // RX: index the next frame is stored at

// Filter control byte: enabled, storing into FIFO n.
#define FLT_ENABLE(fifo)    (0x80u | ((fifo) & 0x1Fu))
#define MASK_SID_LANE       (3u << MCPFD_LANE_SHIFT)

// FIFOCON values we configure, so run-time writes need no read-modify-write.
#define RX_CON(depth)   ((((uint32_t)(depth) - 1) << FIFOCON_FSIZE_SHIFT) | FIFOCON_TFNRFNIE)
#define TX_CON          ((((uint32_t)MCPFD_TX_DEPTH - 1) << FIFOCON_FSIZE_SHIFT) | FIFOCON_TXEN)

// ---------------------------------------------------------------------------
// SPI helpers (caller holds the device lock)
// ---------------------------------------------------------------------------
static inline uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static esp_err_t xfer(mcpfd_stream_t *s, const uint8_t *tx, uint8_t *rx, size_t len)
{
    esp_err_t err = s->xfer(s->io, tx, rx, len);
    if (err != ESP_OK) s->stats.spi_errors++;
    return err;
}

// Read `len` bytes (<= 16) at `addr`; the data is left at s->reg_rx + 2.
static esp_err_t reg_read(mcpfd_stream_t *s, uint16_t addr, size_t len)
{
    uint16_t cmd = SPI_CMD_READ(addr);
    memset(s->reg_tx, 0, 2 + len);
    s->reg_tx[0] = (uint8_t)(cmd >> 8);
    s->reg_tx[1] = (uint8_t)cmd;
    return xfer(s, s->reg_tx, s->reg_rx, 2 + len);
}

static esp_err_t reg_read32(mcpfd_stream_t *s, uint16_t addr, uint32_t *val)
{
    esp_err_t err = reg_read(s, addr, 4);
    if (err == ESP_OK) *val = le32(s->reg_rx + 2);
    return err;
}

static esp_err_t reg_write32(mcpfd_stream_t *s, uint16_t addr, uint32_t val)
{
    uint16_t cmd = SPI_CMD_WRITE(addr);
    s->reg_tx[0] = (uint8_t)(cmd >> 8);
    s->reg_tx[1] = (uint8_t)cmd;
    put_le32(s->reg_tx + 2, val);
    return xfer(s, s->reg_tx, NULL, 6);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void mcpfd_stream_init(mcpfd_stream_t *s)
{
    // RAM follows the FIFO numbers: lane 0 (FIFO 1), TX (FIFO 2), lanes 1-3.
    uint16_t base = 0;
    for (int i = 0; i < MCPFD_LANES; i++) {
        s->lane[i] = (mcpfd_lane_t){ (uint8_t)MCPFD_LANE_FIFO(i), MCPFD_LANE_DEPTH, base, -1 };
        base += MCPFD_LANE_DEPTH * MCPFD_OBJ_SIZE;
        if (i == 0) {
            s->tx_base = base;
            base += MCPFD_TX_DEPTH * MCPFD_OBJ_SIZE;
        }
    }
    // base == MCPFD_RAM_SIZE here (checked by the host test).

    s->next_lane = 0;
    memset(&s->stats, 0, sizeof(s->stats));
}

esp_err_t mcpfd_stream_config_fifos(mcpfd_stream_t *s)
{
    esp_err_t err = reg_write32(s, REG_C1FIFOCON(MCPFD_FIFO_TX), TX_CON);
    for (int i = 0; i < MCPFD_LANES && err == ESP_OK; i++) {
        err = reg_write32(s, REG_C1FIFOCON(s->lane[i].fifo), RX_CON(s->lane[i].depth));
    }
    return err;
}

static esp_err_t filters_disable_all(mcpfd_stream_t *s)
{
    esp_err_t err = ESP_OK;
    for (int w = 0; w < MCPFD_FILTERS / 4 && err == ESP_OK; w++) {
        err = reg_write32(s, REG_C1FLTCON(w), 0);
    }
    return err;
}

esp_err_t mcpfd_stream_filters_accept_all_lane0(mcpfd_stream_t *s)
{
    esp_err_t err = filters_disable_all(s);
    if (err == ESP_OK) err = reg_write32(s, REG_C1FLTOBJ(0), 0);
    if (err == ESP_OK) err = reg_write32(s, REG_C1MASK(0), 0);   // mask 0: every id matches
    if (err == ESP_OK) err = reg_write32(s, REG_C1FLTCON(0), FLT_ENABLE(MCPFD_FIFO_RX0));
    return err;
}

esp_err_t mcpfd_stream_filters_program(mcpfd_stream_t *s)
{
    // A filter object can only be written while its filter is disabled.
    esp_err_t err = filters_disable_all(s);

    // Four filters on two id bits, one lane each: every id matches exactly one of them.
    uint32_t con = 0;
    for (int k = 0; k < MCPFD_LANES && err == ESP_OK; k++) {
        err = reg_write32(s, REG_C1FLTOBJ(k), (uint32_t)k << MCPFD_LANE_SHIFT);
        if (err == ESP_OK) err = reg_write32(s, REG_C1MASK(k), MASK_SID_LANE);
        con |= FLT_ENABLE(MCPFD_LANE_FIFO(k)) << (k * 8);
    }
    if (err == ESP_OK) err = reg_write32(s, REG_C1FLTCON(0), con);
    return err;
}

esp_err_t mcpfd_stream_reset_rx(mcpfd_stream_t *s)
{
    esp_err_t err = ESP_OK;
    for (int i = 0; i < MCPFD_LANES && err == ESP_OK; i++) {
        err = reg_write32(s, REG_C1FIFOCON(s->lane[i].fifo),
                          RX_CON(s->lane[i].depth) | FIFOCON_FRESET);
        s->lane[i].expect_tail = -1;
    }
    return err;
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------
void mcpfd_stream_parse(const uint8_t *obj, can_frame_t *out)
{
    uint32_t r0 = le32(obj);
    uint32_t r1 = le32(obj + 4);

    memset(out, 0, sizeof(*out));
    out->id = r0 & 0x7FF;
    out->extended = (r1 >> 4) & 1;
    if (out->extended) {
        out->id = (out->id << 18) | ((r0 >> 11) & 0x3FFFF);
    }
    out->dlc = r1 & 0x0F;
    out->fd = (r1 >> 7) & 1;
    out->brs = (r1 >> 6) & 1;

    // The FIFOs hold 8 payload bytes per message; a longer CAN FD frame is
    // delivered with its first 8.
    uint8_t len = out->dlc > 8 ? 8 : out->dlc;
    memcpy(out->data, obj + 8, len);
}

// Messages waiting in an RX FIFO whose oldest one is at index `tail`, from one
// FIFOSTA value. head == tail means full if the chip says so; otherwise 0,
// which a FIFO that is not empty cannot hold (the index read wrong).
static int fifo_count(uint32_t sta, int tail, int depth)
{
    int count = ((int)FIFOSTA_FIFOCI(sta) - tail + depth) % depth;
    return (count == 0 && (sta & FIFOSTA_TFERFFIF)) ? depth : count;
}

// Read up to `max` messages from one lane into s->blk_rx + 2 and release them
// on the chip. Returns the number read and released (0: lane empty, or an
// error before anything was released).
static int rx_chunk(mcpfd_stream_t *s, int li, int max)
{
    mcpfd_lane_t *lane = &s->lane[li];

    // FIFOSTA and FIFOUA are adjacent: one read gives the flags, the index the
    // chip stores the next frame at (head) and the RAM address of the oldest
    // unread frame (tail).
    if (reg_read(s, REG_C1FIFOSTA(lane->fifo), 8) != ESP_OK) return 0;
    uint32_t sta = le32(s->reg_rx + 2);
    uint32_t ua  = le32(s->reg_rx + 6);

    if (sta & FIFOSTA_RXOVIF) {
        s->stats.rx_overflow[li]++;
        // Sticky flag: cleared by writing it back as 0.
        reg_write32(s, REG_C1FIFOSTA(lane->fifo), sta & ~FIFOSTA_RXOVIF);
    }
    if (!(sta & FIFOSTA_TFNRFNIF)) {
        lane->expect_tail = -1;
        return 0;
    }

    int depth = lane->depth;
    int head = (int)FIFOSTA_FIFOCI(sta);
    int32_t off = (int32_t)ua - (int32_t)lane->base;

    if (off < 0 || off >= depth * MCPFD_OBJ_SIZE || (off % MCPFD_OBJ_SIZE) != 0 || head >= depth) {
        // Not a state this FIFO can be in: a corrupted read (a chip that is
        // unpowered or in reset reads as all ones) or a layout that is not the
        // one configured. Nothing is delivered and nothing is released.
        s->stats.rx_bad_status++;
        lane->expect_tail = -1;
        return 0;
    }

    int tail = off / MCPFD_OBJ_SIZE;
    if (lane->expect_tail >= 0 && lane->expect_tail != tail) {
        s->stats.rx_desync++;
    }
    int count = fifo_count(sta, tail, depth);
    if (count == 0) {
        // Not empty, yet no frames by the index: it read wrong. One frame is
        // always there.
        count = 1;
        s->stats.rx_head_glitch++;
    } else if (count > 1) {
        // The index can also read too large (MCP2518FD erratum DS80000789E 6),
        // which would deliver frames that were already processed, or are not
        // written yet, and release messages that are not there. Frames only
        // arrive between two reads, so the smaller of two counts is never more
        // than what the FIFO holds, unless both reads are wrong the same way.
        // One frame needs no second read: "not empty" does not come from the index.
        int again = 1;
        if (reg_read(s, REG_C1FIFOSTA(lane->fifo), 4) == ESP_OK) {
            uint32_t sta2 = le32(s->reg_rx + 2);
            int head2 = (int)FIFOSTA_FIFOCI(sta2);
            if ((sta2 & FIFOSTA_TFNRFNIF) && head2 < depth) {
                again = fifo_count(sta2, tail, depth);
                if (again == 0) again = 1;
            }
        }
        if (again < count) {
            s->stats.rx_head_unconfirmed++;
            count = again;
        }
    }
    int run = count;
    if (run > depth - tail) run = depth - tail;   // up to the end of the ring
    if (run > max) run = max;

    size_t bytes = (size_t)run * MCPFD_OBJ_SIZE;
    uint16_t cmd = SPI_CMD_READ(RAM_BASE + ua);
    memset(s->blk_tx, 0, 2 + bytes);
    s->blk_tx[0] = (uint8_t)(cmd >> 8);
    s->blk_tx[1] = (uint8_t)cmd;
    if (xfer(s, s->blk_tx, s->blk_rx, 2 + bytes) != ESP_OK) {
        lane->expect_tail = -1;
        return 0;   // nothing released: the frames are read again next time
    }

    // Release the messages. After a failed increment only the released ones
    // count as read: the rest stay on the chip and are read again next time.
    int done = 0;
    while (done < run &&
           reg_write32(s, REG_C1FIFOCON(lane->fifo), RX_CON(depth) | FIFOCON_UINC) == ESP_OK) {
        done++;
    }
    lane->expect_tail = (done == run) ? (int16_t)((tail + run) % depth) : -1;
    s->stats.rx_frames[li] += (uint32_t)done;
    return done;
}

int mcpfd_stream_step(mcpfd_stream_t *s, const mcpfd_hooks_t *h)
{
    // The chunk is copied out of the SPI buffer before the lock is released, so
    // the buffers are only ever touched under the lock.
    uint8_t objs[MCPFD_CHUNK * MCPFD_OBJ_SIZE];
    uint32_t rxif = 0;
    int n = 0;

    h->lock(h->ctx);
    if (reg_read32(s, REG_C1RXIF, &rxif) == ESP_OK) {
        // The next lane in turn that holds frames, so no lane waits behind another's backlog.
        for (int k = 0; k < MCPFD_LANES; k++) {
            int li = (s->next_lane + k) % MCPFD_LANES;
            if (rxif & (1u << s->lane[li].fifo)) {
                s->next_lane = (uint8_t)((li + 1) % MCPFD_LANES);
                n = rx_chunk(s, li, MCPFD_CHUNK);
                if (n > 0) memcpy(objs, s->blk_rx + 2, (size_t)n * MCPFD_OBJ_SIZE);
                break;
            }
        }
    }
    h->unlock(h->ctx);

    can_frame_t f;
    for (int i = 0; i < n; i++) {
        mcpfd_stream_parse(objs + (size_t)i * MCPFD_OBJ_SIZE, &f);
        h->deliver(h->ctx, &f);
    }
    return n;
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
esp_err_t mcpfd_stream_send(mcpfd_stream_t *s, const can_frame_t *frame)
{
    if (frame->dlc > 8) return ESP_ERR_NOT_SUPPORTED;

    // Flags + address of the next free TX slot in one read.
    esp_err_t err = reg_read(s, REG_C1FIFOSTA(MCPFD_FIFO_TX), 8);
    if (err != ESP_OK) return err;
    uint32_t sta = le32(s->reg_rx + 2);
    uint32_t ua  = le32(s->reg_rx + 6);
    if (!(sta & FIFOSTA_TFNRFNIF)) {
        s->stats.tx_full++;
        return ESP_ERR_NO_MEM;
    }

    uint32_t t0 = frame->id & 0x7FF;
    if (frame->extended) {
        t0 = ((frame->id >> 18) & 0x7FF) | ((frame->id & 0x3FFFF) << 11);
    }
    uint32_t t1 = frame->dlc & 0x0F;
    if (frame->extended) t1 |= (1u << 4);
    if (frame->fd)       t1 |= (1u << 7);
    if (frame->brs)      t1 |= (1u << 6);

    // Always the whole 16-byte slot, as the default driver does.
    uint16_t cmd = SPI_CMD_WRITE(RAM_BASE + ua);
    memset(s->reg_tx, 0, 2 + MCPFD_OBJ_SIZE);
    s->reg_tx[0] = (uint8_t)(cmd >> 8);
    s->reg_tx[1] = (uint8_t)cmd;
    put_le32(s->reg_tx + 2, t0);
    put_le32(s->reg_tx + 6, t1);
    memcpy(s->reg_tx + 10, frame->data, frame->dlc);
    err = xfer(s, s->reg_tx, NULL, 2 + MCPFD_OBJ_SIZE);
    if (err != ESP_OK) return err;

    err = reg_write32(s, REG_C1FIFOCON(MCPFD_FIFO_TX), TX_CON | FIFOCON_UINC | FIFOCON_TXREQ);
    if (err == ESP_OK) s->stats.tx_frames++;
    return err;
}
