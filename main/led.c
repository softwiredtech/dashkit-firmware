#include "led.h"
#include "board.h"
#include "driver/gpio.h"

// Active-low: 0 = on, 1 = off
#define LED_ON  0
#define LED_OFF 1

esp_err_t led_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN_R) | (1ULL << LED_PIN_G) | (1ULL << LED_PIN_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;

    led_set_color(LED_COLOR_OFF);
    return ESP_OK;
}

void led_set_rgb(bool r, bool g, bool b)
{
    gpio_set_level(LED_PIN_R, r ? LED_ON : LED_OFF);
    gpio_set_level(LED_PIN_G, g ? LED_ON : LED_OFF);
    gpio_set_level(LED_PIN_B, b ? LED_ON : LED_OFF);
}

void led_set_color(led_color_t color)
{
    switch (color) {
        case LED_COLOR_OFF:     led_set_rgb(0, 0, 0); break;
        case LED_COLOR_RED:     led_set_rgb(1, 0, 0); break;
        case LED_COLOR_GREEN:   led_set_rgb(0, 1, 0); break;
        case LED_COLOR_BLUE:    led_set_rgb(0, 0, 1); break;
        case LED_COLOR_YELLOW:  led_set_rgb(1, 1, 0); break;
        case LED_COLOR_CYAN:    led_set_rgb(0, 1, 1); break;
        case LED_COLOR_MAGENTA: led_set_rgb(1, 0, 1); break;
        case LED_COLOR_WHITE:   led_set_rgb(1, 1, 1); break;
    }
}
