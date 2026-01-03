#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

#define LED_GPIO 45

// led level, 0= low level on, 1= high level on
#define LED_ACTIVE_LEVEL 1

void led_init(void);
void led_on(void);
void led_off(void);
void led_set(bool on);

#ifdef __cplusplus
}
#endif
