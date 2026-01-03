#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

#define LED_ORANGE 37
#define LED_GREEN 38
#define LED_BLUE 45

// led level, 0= low level on, 1= high level on
#define LED_ACTIVE_LEVEL 1

void led_init(uint32_t gpio);

inline void led_on(uint32_t gpio)
{
    gpio_set_level(gpio, LED_ACTIVE_LEVEL);
}

inline void led_off(uint32_t gpio)
{
    gpio_set_level(gpio, !LED_ACTIVE_LEVEL);
}

inline void led_set(uint32_t gpio, bool on)
{
    gpio_set_level(gpio, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}

#ifdef __cplusplus
}
#endif
