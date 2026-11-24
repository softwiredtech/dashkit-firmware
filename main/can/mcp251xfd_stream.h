#pragma once

// MCP251xFD receive/transmit core.
//
// The chip has 32 hardware filters and 2 KB of message RAM. This core receives the whole bus with them, so
// neither limits which CAN messages the firmware or the app can use:
//
//   * Four filters spread every frame over four receive FIFOs ("lanes") by two bits of its id. Which frames
//     the firmware acts on and which are forwarded to the app is decided in software afterwards (can_filter),
//     so no message list is programmed into the chip.
//   * The lanes are served in turn, one chunk each.
//   * Frames are read in chunks: one status read (a second one to confirm the count when it is more than one
//     frame, because the chip's index can read too large), one RAM read for up to MCPFD_CHUNK messages, one
//     pointer increment per message.
//
// The code here is plain C with no FreeRTOS or ESP-IDF driver dependencies:
// SPI goes through one full-duplex transfer callback and locking is left to the
// caller, so the exact same logic runs against a chip model in the host tests
// (tools/test/test_can_stream.c).

#include "can_interface.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// FIFO layout. Sizes are in messages of 16 bytes (8 header + 8 payload), and
// together they use the chip's 2048 bytes of RAM exactly:
//   31*16 (lane 0) + 4*16 (TX) + 3*31*16 (lanes 1-3) = 2048
// Their order in RAM follows the FIFO numbers. Lane 0 is FIFO 1 and TX is
// FIFO 2, the two FIFOs the loopback self-test uses.
#define MCPFD_FIFO_RX0        1      // RX, lane 0
#define MCPFD_FIFO_TX         2      // TX
#define MCPFD_FIFO_RX1        3      // RX, lanes 1..3: FIFO 3..5
#define MCPFD_LANES           4
#define MCPFD_LANE_FIFO(k)    ((k) == 0 ? MCPFD_FIFO_RX0 : MCPFD_FIFO_RX1 + (k) - 1)
// Lane of an id: bits 4-5 of the standard id (for an extended id, of its top 11 bits). On a Tesla the low id
// bits are far from evenly used (one lane took over half of a bus), which overflowed that lane at peak load
// while the others sat empty; these two split it evenly.
#define MCPFD_LANE_SHIFT      4
#define MCPFD_LANE_OF(id)     (((id) >> MCPFD_LANE_SHIFT) & 3)
#define MCPFD_LANE_DEPTH      31
#define MCPFD_TX_DEPTH        4
#define MCPFD_OBJ_SIZE        16
#define MCPFD_RAM_SIZE        2048

// Messages moved per SPI read. Bounds how long one step holds the SPI lock, so
// a pending transmit waits for at most one chunk.
#define MCPFD_CHUNK           8

// Hardware filters on the chip; one per lane is used.
#define MCPFD_FILTERS         32

// Scratch buffer sizes the caller must provide.
#define MCPFD_REG_BUF_SIZE    24                                   // 2 + 16, padded
#define MCPFD_BLK_BUF_SIZE    (4 + MCPFD_CHUNK * MCPFD_OBJ_SIZE)   // 2 + chunk, padded

// One full-duplex SPI transfer of `len` bytes with chip select held for the
// whole transfer. `rx` may be NULL when the reply is not needed.
typedef esp_err_t (*mcpfd_xfer_fn)(void *io, const uint8_t *tx, uint8_t *rx, size_t len);

typedef struct {
    uint8_t  fifo;        // chip FIFO number
    uint8_t  depth;       // messages
    uint16_t base;        // RAM offset of message 0
    int16_t  expect_tail; // tail index we expect on the next poll, or -1
} mcpfd_lane_t;

typedef struct {
    uint32_t rx_frames[MCPFD_LANES];     // frames read per lane
    uint32_t rx_overflow[MCPFD_LANES];   // overflow events per lane (>= 1 lost frame each)
    uint32_t rx_head_glitch;             // not-empty FIFO reporting head == tail without "full"
    uint32_t rx_head_unconfirmed;        // head index a second read did not confirm (smaller count used)
    uint32_t rx_bad_status;              // FIFO status outside the configured layout: nothing read
    uint32_t rx_desync;                  // tail did not move as far as we incremented it
    uint32_t tx_frames;                  // frames handed to the chip
    uint32_t tx_full;                    // sends refused: TX FIFO full
    uint32_t spi_errors;                 // failed SPI transfers
} mcpfd_stats_t;

typedef struct {
    mcpfd_xfer_fn xfer;
    void         *io;
    // Scratch buffers (DMA-capable on the target). reg_* are used for register
    // access and transmit, blk_* for chunked message reads. All of them are only
    // touched while the caller holds the device lock.
    uint8_t      *reg_tx, *reg_rx;       // MCPFD_REG_BUF_SIZE each
    uint8_t      *blk_tx, *blk_rx;       // MCPFD_BLK_BUF_SIZE each

    mcpfd_lane_t  lane[MCPFD_LANES];
    uint16_t      tx_base;
    uint8_t       next_lane;             // round-robin cursor over the lanes
    mcpfd_stats_t stats;
} mcpfd_stream_t;

// Hooks for mcpfd_stream_step. lock/unlock bracket every group of SPI transfers
// (one chunk at most); deliver runs with the lock released.
typedef struct {
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);
    void (*deliver)(void *ctx, const can_frame_t *frame);
    void *ctx;
} mcpfd_hooks_t;

// Fill in the lane table. xfer/io and the four scratch buffers must be set.
void mcpfd_stream_init(mcpfd_stream_t *s);

// Configuration mode only: size the FIFOs and enable their not-empty interrupt.
esp_err_t mcpfd_stream_config_fifos(mcpfd_stream_t *s);

// Configuration mode only: disable every filter, then route all frames into
// lane 0 (used for the loopback self-test, which reads FIFO 1).
esp_err_t mcpfd_stream_filters_accept_all_lane0(mcpfd_stream_t *s);

// Program the real filters: every frame on the bus, to the lane its id selects, so a given id always lands in
// the same lane and its frames stay in order. Allowed in normal mode too: every filter is disabled before it is
// rewritten, at the cost of the frames that arrive during the register writes.
esp_err_t mcpfd_stream_filters_program(mcpfd_stream_t *s);

// Drop everything waiting in the RX FIFOs.
esp_err_t mcpfd_stream_reset_rx(mcpfd_stream_t *s);

// One service step: check which lanes hold frames, read one chunk from the
// next one in turn, and deliver it. Returns the number of frames delivered;
// 0 means every lane was empty.
// Call it in a loop until it returns 0.
int mcpfd_stream_step(mcpfd_stream_t *s, const mcpfd_hooks_t *hooks);

// Queue one classic CAN frame (dlc <= 8) for transmission. The caller holds the
// device lock. ESP_ERR_NO_MEM: the TX FIFO is full, nothing was written.
esp_err_t mcpfd_stream_send(mcpfd_stream_t *s, const can_frame_t *frame);

// Decode a 16-byte RX message object.
void mcpfd_stream_parse(const uint8_t *obj, can_frame_t *out);
