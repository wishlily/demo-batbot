#include "led.h"

void led_init(uint32_t gpio)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;       // set as output mode
    io_conf.pin_bit_mask = (1ULL << gpio);
    io_conf.pull_down_en = 0; // disable pull-down mode
    io_conf.pull_up_en = 0;   // disable pull-up mode
    gpio_config(&io_conf);
    led_off(gpio); // turn off
}
