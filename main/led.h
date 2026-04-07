#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,
    LED_COLOR_CYAN,
    LED_COLOR_MAGENTA,
    LED_COLOR_WHITE,
} led_color_t;

esp_err_t led_init(void);
void led_set_color(led_color_t color);
void led_set_rgb(bool r, bool g, bool b);
