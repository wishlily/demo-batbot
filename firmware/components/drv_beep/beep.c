#include "beep.h"

#include <stdatomic.h>

#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "BEEP";

static atomic_int g_timer_ms = ATOMIC_VAR_INIT(0);

inline static void beep_on(void)
{
    gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
}

inline static void beep_off(void)
{
    gpio_set_level(BEEP_GPIO, !BEEP_ACTIVE_LEVEL);
}

void beep_init(void)
{
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << BEEP_GPIO);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    beep_off();
}

// Set the buzzer start time. When time=0, the buzzer is turned off.
// When time>=1, the buzzer is turned off automatically after xx milliseconds
void beep_on_time(uint16_t time)
{
    if (time <= 0) {
        beep_off();
        return;
    }
    beep_on();
    atomic_store(&g_timer_ms, time);
    ESP_LOGD(TAG, "beep_on_time: %d", time);
}

void beep_update(uint32_t elapsed_ms)
{
    int timer = atomic_load(&g_timer_ms);
    if (timer <= 0) {
        return;
    }
    timer -= elapsed_ms;
    ESP_LOGD(TAG, "beep_update: %d", timer);

    if (timer <= 0) {
        beep_off();
        atomic_store(&g_timer_ms, 0);
        ESP_LOGD(TAG, "beep off");
    } else {
        atomic_store(&g_timer_ms, timer);
    }
}
