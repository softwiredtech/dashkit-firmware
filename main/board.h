#pragma once

// --- SPI: CAN Interface 0 (MCP2518FDT) ---
#define CAN0_SPI_HOST       SPI2_HOST
#define CAN0_PIN_MOSI       GPIO_NUM_11
#define CAN0_PIN_SCLK       GPIO_NUM_12
#define CAN0_PIN_MISO       GPIO_NUM_13
#define CAN0_PIN_CS         GPIO_NUM_10
#define CAN0_PIN_INT        GPIO_NUM_9
#define CAN0_SPI_CLOCK_HZ   (10 * 1000 * 1000)  // 10 MHz SPI clock
#define CAN0_OSC_FREQ_HZ    (40 * 1000 * 1000)   // 40 MHz crystal

// --- SPI: CAN Interface 1 (future, placeholder) ---
// #define CAN1_SPI_HOST    SPI3_HOST
// #define CAN1_PIN_MOSI    GPIO_NUM_x
// #define CAN1_PIN_SCLK    GPIO_NUM_x
// #define CAN1_PIN_MISO    GPIO_NUM_x
// #define CAN1_PIN_CS      GPIO_NUM_x
// #define CAN1_PIN_INT     GPIO_NUM_x

// --- RGB LED (active low) ---
#define LED_PIN_R           GPIO_NUM_39
#define LED_PIN_G           GPIO_NUM_40
#define LED_PIN_B           GPIO_NUM_41
