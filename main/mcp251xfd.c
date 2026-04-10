#include "mcp251xfd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "mcp251xfd";

// ---------------------------------------------------------------------------
// MCP251xFD Register Addresses
// ---------------------------------------------------------------------------
#define REG_C1CON       0x000
#define REG_C1NBTCFG    0x004
#define REG_C1DBTCFG    0x008
#define REG_C1TDC       0x00C
#define REG_C1INT       0x01C
#define REG_C1RXIF      0x024
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
    esp_err_t err = spi_device_transmit(ctx->spi_dev, &t);
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
    esp_err_t err = spi_device_transmit(ctx->spi_dev, &t);
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
    return spi_device_transmit(ctx->spi_dev, &t);
}

// ---------------------------------------------------------------------------
// DLC helpers
// ---------------------------------------------------------------------------
static uint8_t dlc_to_len(uint8_t dlc)
{
    static const uint8_t map[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return (dlc < 16) ? map[dlc] : 64;
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

static esp_err_t configure_fifos(mcp251xfd_ctx_t *ctx)
{
    // FIFO 1: RX FIFO - 16 messages deep, 8 byte payload
    uint32_t rxfifo_con = (0 << FIFOCON_PLSIZE_SHIFT) |  // 8 bytes payload
                          (15 << FIFOCON_FSIZE_SHIFT) |   // 16 messages (value = n-1)
                          FIFOCON_TFNRFNIE;               // Not-empty interrupt
    esp_err_t err = write_reg32(ctx, REG_C1FIFOCON(1), rxfifo_con);
    if (err != ESP_OK) return err;

    // FIFO 2: TX FIFO - 4 messages deep, 8 byte payload
    uint32_t txfifo_con = (0 << FIFOCON_PLSIZE_SHIFT) |  // 8 bytes payload
                          (3 << FIFOCON_FSIZE_SHIFT)  |   // 4 messages
                          FIFOCON_TXEN;                    // TX mode
    err = write_reg32(ctx, REG_C1FIFOCON(2), txfifo_con);
    if (err != ESP_OK) return err;

    // Filter 0: Accept all frames -> FIFO 1
    write_reg32(ctx, REG_C1FLTOBJ(0), 0x00000000);  // Match any ID
    write_reg32(ctx, REG_C1MASK(0), 0x00000000);     // Don't care about any bits

    // Enable filter 0, point to FIFO 1
    // C1FLTCON0 byte 0: bit7=FLTEN, bits[4:0]=F0BP (FIFO pointer)
    // FIFO 1 = 0x01, Enable = 0x80 → 0x81
    uint32_t fltcon_val = 0x00000081;
    write_reg32(ctx, REG_C1FLTCON(0), fltcon_val);

    return ESP_OK;
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

static void read_rx_fifo(mcp251xfd_ctx_t *ctx)
{
    while (true) {
        // Check FIFO 1 status
        uint32_t fifosta;
        read_reg32(ctx, REG_C1FIFOSTA(1), &fifosta);
        if (!(fifosta & FIFOSTA_TFNRFNIF)) {
            break;  // FIFO empty
        }

        // Get user address (RAM offset of next message)
        uint32_t ua;
        read_reg32(ctx, REG_C1FIFOUA(1), &ua);

        // Read message object from RAM (8 + 8 bytes for header + data)
        uint8_t msg[16];
        spi_read_reg(ctx, RAM_BASE + ua, msg, 16);

        // Parse message object
        uint32_t r0 = msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24);
        uint32_t r1 = msg[4] | (msg[5] << 8) | (msg[6] << 16) | (msg[7] << 24);

        can_tagged_frame_t tagged;
        memset(&tagged, 0, sizeof(tagged));
        tagged.bus_id = ctx->bus_id;

        // SID is in bits [10:0]
        tagged.frame.id = r0 & 0x7FF;
        tagged.frame.extended = (r1 >> 4) & 1;
        if (tagged.frame.extended) {
            uint32_t eid = (r0 >> 11) & 0x3FFFF;
            tagged.frame.id = (tagged.frame.id << 18) | eid;
        }

        tagged.frame.dlc = r1 & 0x0F;
        tagged.frame.fd = (r1 >> 7) & 1;
        tagged.frame.brs = (r1 >> 6) & 1;

        uint8_t data_len = dlc_to_len(tagged.frame.dlc);
        if (data_len > 8) {
            // For CAN FD frames > 8 bytes, read remaining data
            uint8_t extra[64];
            spi_read_reg(ctx, RAM_BASE + ua + 8, extra, data_len);
            memcpy(tagged.frame.data, extra, data_len);
        } else {
            memcpy(tagged.frame.data, &msg[8], data_len);
        }

        // Increment FIFO head pointer
        uint32_t fifocon;
        read_reg32(ctx, REG_C1FIFOCON(1), &fifocon);
        fifocon |= FIFOCON_UINC;
        write_reg32(ctx, REG_C1FIFOCON(1), fifocon);

        // Deliver frame via callback
        if (ctx->rx_cb) {
            ctx->rx_cb(&tagged, ctx->rx_cb_ctx);
        }
    }
}

static void rx_task(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
#ifdef CONFIG_DASHKIT_DEBUG_LOG
    uint32_t poll_count = 0;
#endif

    while (ctx->running) {
        uint32_t intflag;
        read_reg32(ctx, REG_C1INT, &intflag);

        if (intflag & C1INT_RXIF) {
            read_rx_fifo(ctx);
        }

#ifdef CONFIG_DASHKIT_DEBUG_LOG
        if ((poll_count++ % 2000) == 0) {
            uint32_t con, fifosta, diag0, diag1;
            read_reg32(ctx, REG_C1CON, &con);
            read_reg32(ctx, REG_C1FIFOSTA(1), &fifosta);
            read_reg32(ctx, REG_C1BDIAG0, &diag0);
            read_reg32(ctx, REG_C1BDIAG1, &diag1);
            uint8_t opmod = (con >> C1CON_OPMOD_SHIFT) & 0x7;
            ESP_LOGI(TAG, "STATUS OPMOD=%d FIFOSTA=0x%08lx DIAG0=0x%08lx DIAG1=0x%08lx",
                     opmod, (unsigned long)fifosta,
                     (unsigned long)diag0, (unsigned long)diag1);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(1));
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
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ctx->pin_int, gpio_isr_handler, ctx);

    // Create RX task
    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, ctx, 10, &ctx->rx_task_handle, 1);

    // Re-enter config mode to reset FIFOs after loopback tests
    set_mode(ctx, OPMODE_CONFIG);

    // Reset RX FIFO 1
    uint32_t fifocon_reset;
    read_reg32(ctx, REG_C1FIFOCON(1), &fifocon_reset);
    fifocon_reset |= (1 << 10);  // FRESET bit
    write_reg32(ctx, REG_C1FIFOCON(1), fifocon_reset);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Clear all interrupt flags
    write_reg32(ctx, REG_C1INT, C1INT_RXIE);  // Keep RXIE, clear all flags
    write_reg32(ctx, REG_C1BDIAG0, 0);
    write_reg32(ctx, REG_C1BDIAG1, 0);

    // Listen-only mode: passively receives CAN 2.0 and CAN FD frames
    // without sending ACKs or error frames (safe for bus monitoring).
    // To enable TX, change to OPMODE_NORMAL_2_0 (or OPMODE_NORMAL_FD for CAN FD).
    esp_err_t err = set_mode(ctx, OPMODE_LISTEN_ONLY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: failed to enter normal mode", ctx->bus_id);
        return err;
    }

    return ESP_OK;
}

static esp_err_t iface_stop(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;
    ctx->running = false;

    gpio_isr_handler_remove(ctx->pin_int);

    // Back to config mode
    set_mode(ctx, OPMODE_CONFIG);

    ESP_LOGI(TAG, "Bus %d: stopped", ctx->bus_id);
    return ESP_OK;
}

static esp_err_t iface_send(can_interface_t *self, const can_frame_t *frame)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    // Check TX FIFO 2 has space
    uint32_t fifosta;
    read_reg32(ctx, REG_C1FIFOSTA(2), &fifosta);
    if (!(fifosta & FIFOSTA_TFNRFNIF)) {
        return ESP_ERR_NO_MEM;  // TX FIFO full
    }

    // Get user address for next TX slot
    uint32_t ua;
    read_reg32(ctx, REG_C1FIFOUA(2), &ua);

    // Build message object
    uint32_t t0 = frame->id & 0x7FF;
    if (frame->extended) {
        t0 = ((frame->id >> 18) & 0x7FF) | (((frame->id) & 0x3FFFF) << 11);
    }

    uint32_t t1 = frame->dlc & 0x0F;
    if (frame->extended) t1 |= (1 << 4);
    if (frame->fd)       t1 |= (1 << 7);
    if (frame->brs)      t1 |= (1 << 6);

    uint8_t data_len = dlc_to_len(frame->dlc);
    uint8_t msg[8 + 64] = {0};
    msg[0] = t0 & 0xFF;
    msg[1] = (t0 >> 8) & 0xFF;
    msg[2] = (t0 >> 16) & 0xFF;
    msg[3] = (t0 >> 24) & 0xFF;
    msg[4] = t1 & 0xFF;
    msg[5] = (t1 >> 8) & 0xFF;
    msg[6] = (t1 >> 16) & 0xFF;
    msg[7] = (t1 >> 24) & 0xFF;
    memcpy(&msg[8], frame->data, data_len);

    spi_write_reg(ctx, RAM_BASE + ua, msg, 8 + data_len);

    // Set UINC and TXREQ
    uint32_t fifocon;
    read_reg32(ctx, REG_C1FIFOCON(2), &fifocon);
    fifocon |= FIFOCON_UINC | FIFOCON_TXREQ;
    write_reg32(ctx, REG_C1FIFOCON(2), fifocon);

    return ESP_OK;
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

    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 76,  // 8 header + 4 cmd + 64 data
    };

    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
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
        if (ctx->int_sem) vSemaphoreDelete(ctx->int_sem);
        // SPI cleanup would go here
        free(ctx);
    }
    free(iface);
}
