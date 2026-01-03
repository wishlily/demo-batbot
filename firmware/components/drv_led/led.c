#include "led.h"

#include <stdio.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

void led_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;       // set as output mode
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.pull_down_en = 0; // disable pull-down mode
    io_conf.pull_up_en = 0;   // disable pull-up mode
    gpio_config(&io_conf);
    led_off(); // turn off
}

void led_on(void)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL);
}

void led_off(void)
{
    gpio_set_level(LED_GPIO, !LED_ACTIVE_LEVEL);
}

void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}