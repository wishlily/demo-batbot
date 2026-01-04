#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define BEEP_GPIO 46

// Buzzer active level, 0= Active low, 1= Active high
#define BEEP_ACTIVE_LEVEL 1

void beep_init(void);
void beep_on_time(uint16_t time);
void beep_update(uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif
