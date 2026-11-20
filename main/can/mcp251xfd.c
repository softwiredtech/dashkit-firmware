#include "mcp251xfd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "mcp251xfd_stream.h"
#include "esp_heap_caps.h"

static const char *TAG = "mcp251xfd";

// ---------------------------------------------------------------------------
// MCP251xFD Register Addresses
// ---------------------------------------------------------------------------
#define REG_C1CON       0x000
#define REG_C1NBTCFG    0x004
#define REG_C1DBTCFG    0x008
#define REG_C1TDC       0x00C
#define REG_C1INT       0x01C
#define REG_C1RXIF      0x020
#define REG_C1TXREQ     0x030
#define REG_C1TEFCON    0x040
#define REG_C1TXQCON    0x050
#define REG_C1TXQSTA    0x054
#define REG_C1TXQUA     0x058
#define REG_C1FIFOCON(n)  (0x05C + ((n) - 1) * 12)
#define REG_C1FIFOSTA(n)  (0x060 + ((n) - 1) * 12)
#define REG_C1FIFOUA(n)   (0x064 + ((n) - 1) * 12)
#define REG_C1FLTCON(n)   (0x1D0 + (n) * 4)
#define REG_C1FLTOBJ(n)   (0x1F0 + (n) * 8)
#define REG_C1MASK(n)     (0x1F4 + (n) * 8)
#define REG_C1TREC      0x034
#define REG_C1BDIAG0    0x038
#define REG_C1BDIAG1    0x03C
#define REG_OSC         0xE00
#define REG_IOCON       0xE04

#define RAM_BASE        0x400

// SPI instructions (4-bit opcode in upper nibble of first byte)
#define SPI_CMD_RESET       0x0000
#define SPI_CMD_READ(addr)  (0x3000 | ((addr) & 0xFFF))
#define SPI_CMD_WRITE(addr) (0x2000 | ((addr) & 0xFFF))

// C1CON bits
#define C1CON_REQOP_SHIFT   24
#define C1CON_REQOP_MASK    (0x7 << C1CON_REQOP_SHIFT)
#define C1CON_OPMOD_SHIFT   21
#define C1CON_OPMOD_MASK    (0x7 << C1CON_OPMOD_SHIFT)
#define C1CON_TXQEN         (1 << 20)
#define C1CON_STEF          (1 << 19)
#define C1CON_ISOCRCEN      (1 << 5)
#define C1CON_PXEDIS        (1 << 6)

#define OPMODE_CONFIG       0x04
#define OPMODE_INT_LOOPBACK 0x02  // Internal loopback mode
#define OPMODE_LISTEN_ONLY  0x03  // Listen only mode
#define OPMODE_EXT_LOOPBACK 0x05  // External loopback mode
#define OPMODE_NORMAL_2_0   0x06  // CAN 2.0 mode
#define OPMODE_NORMAL_FD    0x00  // CAN FD mode

// C1INT bits
#define C1INT_RXIE          (1 << 17)
#define C1INT_RXIF          (1 << 1)

// FIFO control bits
#define FIFOCON_TXEN        (1 << 7)
#define FIFOCON_UINC        (1 << 8)
#define FIFOCON_TXREQ       (1 << 9)
#define FIFOCON_TFNRFNIE    (1 << 0)   // RX not empty / TX not full interrupt enable
#define FIFOCON_FSIZE_SHIFT 24
#define FIFOCON_PLSIZE_SHIFT 29

// FIFO status bits
#define FIFOSTA_TFNRFNIF    (1 << 0)   // Not empty (RX) / not full (TX)
#define FIFOSTA_RXOVIF      (1 << 3)   // RX FIFO overflow: a frame was dropped
                                       // because the FIFO was full (sticky; the
                                       // app clears it by writing the bit to 0)

// How often the rx_task reports the accumulated RX FIFO overflow count.
#define RX_OVERFLOW_REPORT_US   (5 * 1000 * 1000)

// OSC register bits
#define OSC_OSCRDY          (1 << 10)
#define OSC_SCLKRDY         (1 << 12)

// ---------------------------------------------------------------------------
// Driver context
// ---------------------------------------------------------------------------
typedef struct {
    spi_device_handle_t spi_dev;
    gpio_num_t          pin_int;
    uint32_t            osc_freq_hz;
    uint32_t            bitrate;
    uint32_t            bitrate_data;
    uint8_t             bus_id;
    can_rx_callback_t   rx_cb;
    void               *rx_cb_ctx;
    TaskHandle_t        rx_task_handle;
    SemaphoreHandle_t   int_sem;
    SemaphoreHandle_t   spi_mutex;
    volatile bool       running;
    esp_timer_handle_t  overflow_timer;
    // Receive/transmit core (mcp251xfd_stream.c). spi_mutex guards
    // each group of SPI transfers; spi_gate makes the hand-over fair, so a
    // transmit waiting for the bus goes next even while the RX task is draining
    // back to back.
    mcpfd_stream_t      stream;
    SemaphoreHandle_t   spi_gate;
    volatile bool       stream_ready;      // filters programmed, chip in normal mode
    mcpfd_stats_t       stats_reported;    // snapshot at the last report
    volatile uint32_t   step_max_us;       // longest single service step
    volatile uint32_t   busy_gap_max_us;   // longest gap between steps while frames were waiting
} mcp251xfd_ctx_t;

// ---------------------------------------------------------------------------
// SPI low-level
// ---------------------------------------------------------------------------
static esp_err_t spi_write_reg(mcp251xfd_ctx_t *ctx, uint16_t addr, const uint8_t *data, size_t len)
{
    uint16_t cmd = SPI_CMD_WRITE(addr);
    uint8_t hdr[2] = { cmd >> 8, cmd & 0xFF };

    spi_transaction_t t = {
        .length = (2 + len) * 8,
        .tx_buffer = NULL,
    };

    uint8_t buf[2 + 64];
    if ((2 + len) > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    memcpy(buf, hdr, 2);
    memcpy(buf + 2, data, len);
    t.tx_buffer = buf;

    xSemaphoreTake(ctx->spi_mutex, portMAX_DELAY);
    esp_err_t err = spi_device_polling_transmit(ctx->spi_dev, &t);
    xSemaphoreGive(ctx->spi_mutex);
    return err;
}

static esp_err_t spi_read_reg(mcp251xfd_ctx_t *ctx, uint16_t addr, uint8_t *data, size_t len)
{
    uint16_t cmd = SPI_CMD_READ(addr);
    uint8_t tx_buf[2 + 64] = {0};
    uint8_t rx_buf[2 + 64] = {0};

    if ((2 + len) > sizeof(tx_buf)) return ESP_ERR_INVALID_SIZE;

    tx_buf[0] = cmd >> 8;
    tx_buf[1] = cmd & 0xFF;

    spi_transaction_t t = {
        .length = (2 + len) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    xSemaphoreTake(ctx->spi_mutex, portMAX_DELAY);
    esp_err_t err = spi_device_polling_transmit(ctx->spi_dev, &t);
    xSemaphoreGive(ctx->spi_mutex);
    if (err == ESP_OK) {
        memcpy(data, rx_buf + 2, len);
    }
    return err;
}

static esp_err_t write_reg32(mcp251xfd_ctx_t *ctx, uint16_t addr, uint32_t val)
{
    uint8_t d[4] = { val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF };
    return spi_write_reg(ctx, addr, d, 4);
}

static esp_err_t read_reg32(mcp251xfd_ctx_t *ctx, uint16_t addr, uint32_t *val)
{
    uint8_t d[4];
    esp_err_t err = spi_read_reg(ctx, addr, d, 4);
    if (err == ESP_OK) {
        *val = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24);
    }
    return err;
}

static esp_err_t spi_reset(mcp251xfd_ctx_t *ctx)
{
    uint8_t tx[2] = {0x00, 0x00};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(ctx->spi_dev, &t);
}

// ---------------------------------------------------------------------------
// Glue: SPI transfer, locking and frame delivery for the core in
// mcp251xfd_stream.c
// ---------------------------------------------------------------------------
static esp_err_t stream_xfer(void *io, const uint8_t *tx, uint8_t *rx, size_t len)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)io;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(ctx->spi_dev, &t);
}

static void stream_lock(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    // Whoever holds the gate is next in line for the bus. The RX task has to
    // pass through it between chunks, so it queues behind a waiting transmit
    // instead of re-taking the bus straight away.
    xSemaphoreTake(ctx->spi_gate, portMAX_DELAY);
    xSemaphoreTake(ctx->spi_mutex, portMAX_DELAY);
    xSemaphoreGive(ctx->spi_gate);
}

static void stream_unlock(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    xSemaphoreGive(ctx->spi_mutex);
}

static void stream_deliver(void *arg, const can_frame_t *frame)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    if (!ctx->rx_cb) return;
    can_tagged_frame_t tagged;
    tagged.frame = *frame;
    tagged.bus_id = ctx->bus_id;
    tagged.timestamp_us = 0;
    ctx->rx_cb(&tagged, ctx->rx_cb_ctx);
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static esp_err_t wait_for_osc(mcp251xfd_ctx_t *ctx)
{
    for (int i = 0; i < 100; i++) {
        uint32_t osc = 0;
        read_reg32(ctx, REG_OSC, &osc);
        if (osc & OSC_OSCRDY) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "Oscillator not ready");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t set_mode(mcp251xfd_ctx_t *ctx, uint8_t mode)
{
    uint32_t con;
    esp_err_t err = read_reg32(ctx, REG_C1CON, &con);
    if (err != ESP_OK) return err;

    con &= ~C1CON_REQOP_MASK;
    con |= ((uint32_t)mode << C1CON_REQOP_SHIFT);
    err = write_reg32(ctx, REG_C1CON, con);
    if (err != ESP_OK) return err;

    for (int i = 0; i < 100; i++) {
        read_reg32(ctx, REG_C1CON, &con);
        uint8_t opmod = (con & C1CON_OPMOD_MASK) >> C1CON_OPMOD_SHIFT;
        if (opmod == mode) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGE(TAG, "Failed to enter mode 0x%02x", mode);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t configure_bitrate(mcp251xfd_ctx_t *ctx)
{
    // Target: 20 TQ per bit, 80% sample point (industry standard)
    // Bitrate = osc_freq / ((BRP+1) * (Sync + TSEG1 + TSEG2))
    // Sync = 1 TQ always

    uint32_t target_tq = 20;

    // Find BRP that gives us close to target_tq
    uint32_t brp = (ctx->osc_freq_hz / (ctx->bitrate * target_tq)) - 1;

    // Verify
    uint32_t actual_tq = ctx->osc_freq_hz / (ctx->bitrate * (brp + 1));
    if (actual_tq < 8 || actual_tq > 40) {
        ESP_LOGE(TAG, "Cannot find valid bit timing for %lu bps", (unsigned long)ctx->bitrate);
        return ESP_ERR_INVALID_ARG;
    }

    // 80% sample point: sample at Sync + TSEG1
    uint8_t tseg1 = (actual_tq * 80 / 100) - 1;
    uint8_t tseg2 = actual_tq - tseg1 - 1;
    uint8_t sjw = tseg2;  // SJW = TSEG2 for maximum tolerance

    // C1NBTCFG: [31:24]=BRP, [23:16]=TSEG1, [14:8]=TSEG2, [6:0]=SJW (all n-1)
    uint32_t nbtcfg = ((sjw - 1) & 0x7F) |
                      (((uint32_t)(tseg2 - 1) & 0x7F) << 8) |
                      (((uint32_t)(tseg1 - 1) & 0xFF) << 16) |
                      (((uint32_t)brp & 0xFF) << 24);

    esp_err_t err = write_reg32(ctx, REG_C1NBTCFG, nbtcfg);
    if (err != ESP_OK) return err;

    // Data bitrate for CAN FD (default to 2Mbps if not specified)
    uint32_t data_rate = ctx->bitrate_data > 0 ? ctx->bitrate_data : (ctx->bitrate * 4);
    uint32_t dbrp = (ctx->osc_freq_hz / (data_rate * 10)) - 1;  // ~10 TQ
    uint32_t dactual_tq = ctx->osc_freq_hz / (data_rate * (dbrp + 1));
    if (dactual_tq >= 5 && dactual_tq <= 25) {
        uint8_t dtseg1 = (dactual_tq * 75 / 100) - 1;
        uint8_t dtseg2 = dactual_tq - dtseg1 - 1;
        uint8_t dsjw = dtseg2;

        // C1DBTCFG: [31:24]=DBRP, [20:16]=DTSEG1, [11:8]=DTSEG2, [3:0]=DSJW (all n-1)
        uint32_t dbtcfg = ((dsjw - 1) & 0x0F) |
                          (((uint32_t)(dtseg2 - 1) & 0x0F) << 8) |
                          (((uint32_t)(dtseg1 - 1) & 0x1F) << 16) |
                          (((uint32_t)dbrp & 0xFF) << 24);

        write_reg32(ctx, REG_C1DBTCFG, dbtcfg);
    }

    return ESP_OK;
}

// ---- Hardware acceptance filtering ----
// The bus is always received in full, spread over the receive lanes by id. Which frames the firmware acts on
// and which go to the app is decided in software (can_filter). The caller holds the device lock.
static esp_err_t program_filters(mcp251xfd_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Bus %u: receiving the whole bus", (unsigned)ctx->bus_id);
    return mcpfd_stream_filters_program(&ctx->stream);
}

static esp_err_t configure_fifos(mcp251xfd_ctx_t *ctx)
{
    // RX FIFO 1, TX FIFO 2, RX FIFOs 3..5. Until iface_start programs the real
    // filters, everything is routed to FIFO 1 so the loopback self-test (which
    // reads FIFO 1) works unchanged.
    stream_lock(ctx);
    esp_err_t serr = mcpfd_stream_config_fifos(&ctx->stream);
    if (serr == ESP_OK) serr = mcpfd_stream_filters_accept_all_lane0(&ctx->stream);
    stream_unlock(ctx);
    return serr;
}

// ---------------------------------------------------------------------------
// Interrupt handling
// ---------------------------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(ctx->int_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


// Receive statistics, checked every 5 s. A healthy bus logs nothing: the line is printed only when frames
// were lost (a lane overflowed) or a send was refused in the period.
static void overflow_report_cb(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    mcpfd_stats_t now = ctx->stream.stats;       // racy snapshot, counters only
    const mcpfd_stats_t *was = &ctx->stats_reported;
    uint32_t rx = 0, rx_was = 0, ov = 0, ov_was = 0;
    for (int i = 0; i < MCPFD_LANES; i++) {
        rx += now.rx_frames[i];
        rx_was += was->rx_frames[i];
        ov += now.rx_overflow[i];
        ov_was += was->rx_overflow[i];
    }
    uint32_t secs = (uint32_t)(RX_OVERFLOW_REPORT_US / 1000000);
    uint32_t step_max = ctx->step_max_us, gap_max = ctx->busy_gap_max_us;
    ctx->step_max_us = 0;
    ctx->busy_gap_max_us = 0;
    if (ov != ov_was || now.tx_full != was->tx_full) {
        ESP_LOGI(TAG, "bus%u stream: rx %lu fps, overflows %lu (+%lu; per lane +%lu +%lu +%lu +%lu) "
                      "tx %lu (full %lu) step max %lu us, busy gap max %lu us",
                 ctx->bus_id,
                 (unsigned long)((rx - rx_was) / secs),
                 (unsigned long)ov, (unsigned long)(ov - ov_was),
                 (unsigned long)(now.rx_overflow[0] - was->rx_overflow[0]),
                 (unsigned long)(now.rx_overflow[1] - was->rx_overflow[1]),
                 (unsigned long)(now.rx_overflow[2] - was->rx_overflow[2]),
                 (unsigned long)(now.rx_overflow[3] - was->rx_overflow[3]),
                 (unsigned long)now.tx_frames, (unsigned long)now.tx_full,
                 (unsigned long)step_max, (unsigned long)gap_max);
    }
    if (now.rx_head_glitch != was->rx_head_glitch || now.rx_head_unconfirmed != was->rx_head_unconfirmed ||
        now.rx_bad_status != was->rx_bad_status || now.rx_desync != was->rx_desync ||
        now.spi_errors != was->spi_errors) {
        ESP_LOGW(TAG, "bus%u stream anomalies: head stale %lu, head unconfirmed %lu, "
                      "bad status %lu, desync %lu, spi errors %lu",
                 ctx->bus_id,
                 (unsigned long)now.rx_head_glitch, (unsigned long)now.rx_head_unconfirmed,
                 (unsigned long)now.rx_bad_status,
                 (unsigned long)now.rx_desync, (unsigned long)now.spi_errors);
    }
    ctx->stats_reported = now;
}

// How long the RX task may drain without a break before it sleeps one tick, so
// lower-priority work on its core (the idle task, the watchdog) still runs on a
// bus that never goes quiet.
#define STREAM_MAX_BUSY_US   (500 * 1000)

static void rx_task(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    const mcpfd_hooks_t hooks = {
        .lock = stream_lock,
        .unlock = stream_unlock,
        .deliver = stream_deliver,
        .ctx = ctx,
    };

    while (ctx->running) {
        // Woken by the INT falling edge (any RX FIFO not empty). The 2 ms
        // timeout covers a missed edge: INT is level-driven and stays low
        // while frames are waiting.
        xSemaphoreTake(ctx->int_sem, pdMS_TO_TICKS(2));
        if (!ctx->stream_ready) continue;

        int64_t burst_start = esp_timer_get_time();
        int64_t prev = 0;
        for (;;) {
            int64_t t0 = esp_timer_get_time();
            if (prev && (uint32_t)(t0 - prev) > ctx->busy_gap_max_us) {
                ctx->busy_gap_max_us = (uint32_t)(t0 - prev);
            }
            // One chunk from the next lane that holds frames.
            int n = mcpfd_stream_step(&ctx->stream, &hooks);
            int64_t t1 = esp_timer_get_time();
            if ((uint32_t)(t1 - t0) > ctx->step_max_us) {
                ctx->step_max_us = (uint32_t)(t1 - t0);
            }
            if (n == 0 || !ctx->running) break;
            prev = t0;
            if (t1 - burst_start > STREAM_MAX_BUSY_US) {
                vTaskDelay(1);
                burst_start = esp_timer_get_time();
                prev = 0;
            } else {
                taskYIELD();   // let the other bus's RX task in between chunks
            }
        }
    }

    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Interface implementation
// ---------------------------------------------------------------------------
static esp_err_t iface_init(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    // Reset the controller
    esp_err_t err = spi_reset(ctx);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Wait for oscillator ready
    err = wait_for_osc(ctx);
    if (err != ESP_OK) return err;

    // Verify we're in configuration mode after reset
    err = set_mode(ctx, OPMODE_CONFIG);
    if (err != ESP_OK) return err;

    // Configure bit timing
    err = configure_bitrate(ctx);
    if (err != ESP_OK) return err;

    // Enable ISO CRC for CAN FD, use default protocol exception handling
    uint32_t con;
    read_reg32(ctx, REG_C1CON, &con);
    con |= C1CON_ISOCRCEN;
    con &= ~C1CON_PXEDIS;  // Allow protocol exception (graceful CAN FD handling)
    con &= ~C1CON_STEF;    // Disable TEF
    con &= ~C1CON_TXQEN;   // Disable TXQ, we use FIFO 2 for TX
    write_reg32(ctx, REG_C1CON, con);

    // Configure FIFOs and filters
    err = configure_fifos(ctx);
    if (err != ESP_OK) return err;

    // Enable RX interrupt in C1INT
    uint32_t intcon;
    read_reg32(ctx, REG_C1INT, &intcon);
    intcon |= C1INT_RXIE;
    write_reg32(ctx, REG_C1INT, intcon);

    ESP_LOGI(TAG, "Bus %d: MCP251xFD initialized (bitrate=%lu)", ctx->bus_id, (unsigned long)ctx->bitrate);
    return ESP_OK;
}

static esp_err_t loopback_self_test(mcp251xfd_ctx_t *ctx)
{
    // Internal + external loopback self-test

    // Enter internal loopback mode
    esp_err_t err = set_mode(ctx, OPMODE_INT_LOOPBACK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Loopback: failed to enter loopback mode");
        return err;
    }

    // Transmit a test frame via FIFO 2
    uint32_t fifosta;
    read_reg32(ctx, REG_C1FIFOSTA(2), &fifosta);
    if (!(fifosta & FIFOSTA_TFNRFNIF)) {
        ESP_LOGE(TAG, "Loopback: TX FIFO full");
        return ESP_FAIL;
    }

    uint32_t ua;
    read_reg32(ctx, REG_C1FIFOUA(2), &ua);

    // Build test frame: ID=0x123, DLC=4, data=0xDE 0xAD 0xBE 0xEF
    uint8_t msg[16] = {0};
    msg[0] = 0x23; msg[1] = 0x01;  // SID = 0x123
    msg[4] = 0x04;                  // DLC = 4
    msg[8] = 0xDE; msg[9] = 0xAD; msg[10] = 0xBE; msg[11] = 0xEF;

    spi_write_reg(ctx, RAM_BASE + ua, msg, 16);

    // Request transmit
    uint32_t fifocon;
    read_reg32(ctx, REG_C1FIFOCON(2), &fifocon);
    fifocon |= FIFOCON_UINC | FIFOCON_TXREQ;
    write_reg32(ctx, REG_C1FIFOCON(2), fifocon);

    // Wait for RX
    vTaskDelay(pdMS_TO_TICKS(50));

    // Check RX FIFO
    read_reg32(ctx, REG_C1FIFOSTA(1), &fifosta);
    if (!(fifosta & FIFOSTA_TFNRFNIF)) {
        uint32_t diag0, diag1;
        read_reg32(ctx, REG_C1BDIAG0, &diag0);
        read_reg32(ctx, REG_C1BDIAG1, &diag1);
        ESP_LOGE(TAG, "Loopback: no frame received! DIAG0=0x%08lx DIAG1=0x%08lx",
                 (unsigned long)diag0, (unsigned long)diag1);
        set_mode(ctx, OPMODE_CONFIG);
        return ESP_FAIL;
    }

    // Read received frame
    read_reg32(ctx, REG_C1FIFOUA(1), &ua);
    uint8_t rx[16];
    spi_read_reg(ctx, RAM_BASE + ua, rx, 16);

    uint32_t r0 = rx[0] | (rx[1] << 8) | (rx[2] << 16) | (rx[3] << 24);
    uint16_t sid = r0 & 0x7FF;
    uint8_t dlc = rx[4] & 0x0F;

    // Increment FIFO pointer
    read_reg32(ctx, REG_C1FIFOCON(1), &fifocon);
    fifocon |= FIFOCON_UINC;
    write_reg32(ctx, REG_C1FIFOCON(1), fifocon);

    if (sid == 0x123 && dlc == 4 && rx[8] == 0xDE && rx[9] == 0xAD) {
        ESP_LOGI(TAG, "Loopback self-test PASSED");
    } else {
        ESP_LOGE(TAG, "Internal loopback FAILED: unexpected data");
        set_mode(ctx, OPMODE_CONFIG);
        return ESP_FAIL;
    }

    // Now test external loopback (through the transceiver)
    set_mode(ctx, OPMODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Clear diagnostics
    write_reg32(ctx, REG_C1BDIAG0, 0);
    write_reg32(ctx, REG_C1BDIAG1, 0);

    set_mode(ctx, OPMODE_EXT_LOOPBACK);

    // TX a frame
    read_reg32(ctx, REG_C1FIFOUA(2), &ua);
    memset(msg, 0, sizeof(msg));
    msg[0] = 0x56; msg[1] = 0x02;  // SID = 0x256
    msg[4] = 0x02;                  // DLC = 2
    msg[8] = 0xCA; msg[9] = 0xFE;

    spi_write_reg(ctx, RAM_BASE + ua, msg, 16);
    read_reg32(ctx, REG_C1FIFOCON(2), &fifocon);
    fifocon |= FIFOCON_UINC | FIFOCON_TXREQ;
    write_reg32(ctx, REG_C1FIFOCON(2), fifocon);

    vTaskDelay(pdMS_TO_TICKS(50));

    // Check result
    read_reg32(ctx, REG_C1FIFOSTA(1), &fifosta);

    if (fifosta & FIFOSTA_TFNRFNIF) {
        // External loopback OK - consume the frame
        read_reg32(ctx, REG_C1FIFOUA(1), &ua);
        spi_read_reg(ctx, RAM_BASE + ua, rx, 16);

        read_reg32(ctx, REG_C1FIFOCON(1), &fifocon);
        fifocon |= FIFOCON_UINC;
        write_reg32(ctx, REG_C1FIFOCON(1), fifocon);
    } else {
        ESP_LOGE(TAG, "External loopback FAILED");
    }

    // Back to config
    set_mode(ctx, OPMODE_CONFIG);
    return ESP_OK;
}

static esp_err_t iface_start(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    // Run loopback self-test first
    loopback_self_test(ctx);

    ctx->running = true;

    // Install GPIO ISR for INT pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ctx->pin_int),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // INT is active low
    };
    gpio_config(&io_conf);
    // ESP_ERR_INVALID_STATE = service already installed by an earlier interface; that's fine.
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(isr_err));
    }
    gpio_isr_handler_add(ctx->pin_int, gpio_isr_handler, ctx);

    // Create RX task
    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, ctx, 10, &ctx->rx_task_handle, 1);

    // Independent periodic reporter for RX FIFO overflow drops. Runs in the
    // esp_timer task so it fires even when the rx_task is pinned draining a busy
    // FIFO.
    const esp_timer_create_args_t ov_args = {
        .callback = overflow_report_cb,
        .arg = ctx,
        .name = "can_rxov",
    };
    if (esp_timer_create(&ov_args, &ctx->overflow_timer) == ESP_OK) {
        esp_timer_start_periodic(ctx->overflow_timer, RX_OVERFLOW_REPORT_US);
    }

    // Re-enter config mode to reset FIFOs after loopback tests
    set_mode(ctx, OPMODE_CONFIG);

    // Reset RX FIFO 1
    uint32_t fifocon_reset;
    read_reg32(ctx, REG_C1FIFOCON(1), &fifocon_reset);
    fifocon_reset |= (1 << 10);  // FRESET bit
    write_reg32(ctx, REG_C1FIFOCON(1), fifocon_reset);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Still in configuration mode: replace the self-test's "everything to FIFO 1" filter with the real routing
    // (every frame to the lane its id selects) and empty every RX FIFO.
    stream_lock(ctx);
    esp_err_t ferr = program_filters(ctx);
    if (ferr == ESP_OK) ferr = mcpfd_stream_reset_rx(&ctx->stream);
    stream_unlock(ctx);
    if (ferr != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: filter setup failed: %s", ctx->bus_id, esp_err_to_name(ferr));
        return ferr;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    // Clear all interrupt flags
    write_reg32(ctx, REG_C1INT, C1INT_RXIE);  // Keep RXIE, clear all flags
    write_reg32(ctx, REG_C1BDIAG0, 0);
    write_reg32(ctx, REG_C1BDIAG1, 0);

    // Normal CAN 2.0 mode: receives frames and is allowed to transmit
    // (ACKs incoming frames and sends queued TX frames). Required for the
    // BLE CAN TX characteristic to actually put frames on the bus.
    // For passive monitoring only, change to OPMODE_LISTEN_ONLY.
    esp_err_t err = set_mode(ctx, OPMODE_NORMAL_2_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: failed to enter normal mode", ctx->bus_id);
        return err;
    }

    ctx->stream_ready = true;
    return ESP_OK;
}

static esp_err_t iface_stop(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;
    ctx->running = false;
    ctx->stream_ready = false;

    gpio_isr_handler_remove(ctx->pin_int);

    // Back to config mode
    set_mode(ctx, OPMODE_CONFIG);

    ESP_LOGI(TAG, "Bus %d: stopped", ctx->bus_id);
    return ESP_OK;
}

static esp_err_t iface_send(can_interface_t *self, const can_frame_t *frame)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    // Three SPI transfers under one lock. The lock is handed over fairly, so
    // this waits for at most the RX chunk in progress.
    stream_lock(ctx);
    esp_err_t serr = mcpfd_stream_send(&ctx->stream, frame);
    stream_unlock(ctx);
    return serr;
}

static void iface_set_rx_callback(can_interface_t *self, can_rx_callback_t cb, void *user_ctx)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;
    ctx->rx_cb = cb;
    ctx->rx_cb_ctx = user_ctx;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
// The semaphores and scratch buffers of a context, and the context itself.
static void free_ctx(mcp251xfd_ctx_t *ctx)
{
    if (ctx->int_sem) vSemaphoreDelete(ctx->int_sem);
    if (ctx->spi_mutex) vSemaphoreDelete(ctx->spi_mutex);
    if (ctx->spi_gate) vSemaphoreDelete(ctx->spi_gate);
    heap_caps_free(ctx->stream.reg_tx);
    heap_caps_free(ctx->stream.reg_rx);
    heap_caps_free(ctx->stream.blk_tx);
    heap_caps_free(ctx->stream.blk_rx);
    free(ctx);
}

can_interface_t *mcp251xfd_create(const mcp251xfd_config_t *config)
{
    can_interface_t *iface = calloc(1, sizeof(can_interface_t));
    mcp251xfd_ctx_t *ctx = calloc(1, sizeof(mcp251xfd_ctx_t));
    if (!iface || !ctx) {
        free(iface);
        free(ctx);
        return NULL;
    }

    ctx->pin_int = config->pin_int;
    ctx->osc_freq_hz = config->osc_freq_hz;
    ctx->bitrate = config->bitrate;
    ctx->bitrate_data = config->bitrate_data;
    ctx->bus_id = config->bus_id;
    ctx->rx_cb = NULL;
    ctx->rx_cb_ctx = NULL;
    ctx->running = false;
    ctx->int_sem = xSemaphoreCreateBinary();
    ctx->spi_mutex = xSemaphoreCreateMutex();
    ctx->spi_gate = xSemaphoreCreateMutex();
    ctx->stream.xfer = stream_xfer;
    ctx->stream.io = ctx;
    // DMA-capable, word-aligned scratch buffers: the SPI driver uses them
    // directly instead of allocating a bounce buffer per transfer.
    ctx->stream.reg_tx = heap_caps_malloc(MCPFD_REG_BUF_SIZE, MALLOC_CAP_DMA);
    ctx->stream.reg_rx = heap_caps_malloc(MCPFD_REG_BUF_SIZE, MALLOC_CAP_DMA);
    ctx->stream.blk_tx = heap_caps_malloc(MCPFD_BLK_BUF_SIZE, MALLOC_CAP_DMA);
    ctx->stream.blk_rx = heap_caps_malloc(MCPFD_BLK_BUF_SIZE, MALLOC_CAP_DMA);
    if (!ctx->spi_gate || !ctx->stream.reg_tx || !ctx->stream.reg_rx ||
        !ctx->stream.blk_tx || !ctx->stream.blk_rx) {
        ESP_LOGE(TAG, "stream buffers: out of memory");
        free_ctx(ctx);
        free(iface);
        return NULL;
    }
    mcpfd_stream_init(&ctx->stream);

    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 160, // chunked RX read: 2 cmd + 8 * 16 data, with margin
    };

    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    // ESP_ERR_INVALID_STATE = bus already initialized by an earlier device on the same host.
    // Sharing the SPI bus across multiple MCP2518FDs (different CS) is supported.
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        free(ctx);
        free(iface);
        return NULL;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
    };

    err = spi_bus_add_device(config->spi_host, &dev_cfg, &ctx->spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        spi_bus_free(config->spi_host);
        free(ctx);
        free(iface);
        return NULL;
    }

    iface->init = iface_init;
    iface->start = iface_start;
    iface->stop = iface_stop;
    iface->send = iface_send;
    iface->set_rx_callback = iface_set_rx_callback;
    iface->bus_id = config->bus_id;
    iface->ctx = ctx;

    return iface;
}

void mcp251xfd_destroy(can_interface_t *iface)
{
    if (!iface) return;
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)iface->ctx;
    if (ctx) {
        if (ctx->running) iface->stop(iface);
        // SPI cleanup would go here
        free_ctx(ctx);
    }
    free(iface);
}
