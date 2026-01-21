#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Motor pin pin definition
#define PWM_GPIO_M1A 4
#define PWM_GPIO_M1B 5

#define PWM_GPIO_M2A 15
#define PWM_GPIO_M2B 16

#define PWM_GPIO_M3A 9
#define PWM_GPIO_M3B 10

#define PWM_GPIO_M4A 13
#define PWM_GPIO_M4B 14

// PWM motor clock frequency, 10MHz, 1 tick = 0.1us
#define PWM_MOTOR_TIMER_RESOLUTION_HZ 10000000

// PWM motor control frequency, 25KHz
#define PWM_MOTOR_FREQ_HZ 25000

// PWM Theoretical maximum (400)
#define PWM_MOTOR_DUTY_TICK_MAX (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)

// Motor dead zone filtering
#define PWM_MOTOR_DEAD_ZONE (200)

// Maximum motor PWM input value
#define PWM_MOTOR_MAX_VALUE (PWM_MOTOR_DUTY_TICK_MAX - PWM_MOTOR_DEAD_ZONE)

// Motor timer group ID
#define PWM_MOTOR_TIMER_GROUP_ID_M1 (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M2 (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M3 (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M4 (1)

// Motor ID number
typedef enum _motor_id {
    MOTOR_ID_M1,
    MOTOR_ID_M2,
    MOTOR_ID_M3,
    MOTOR_ID_M4,
    MOTOR_ID_MAX, // ALL MOTORS
    MOTOR_ID_ALL = MOTOR_ID_MAX
} motor_id_t;

// Motor stop mode
typedef enum _stop_mode {
    STOP_COAST = 0,
    STOP_BRAKE = 1,
} stop_mode_t;

esp_err_t pwmmotor_init(void);
esp_err_t pwmmotor_set_speed(motor_id_t motor_id, int speed);
esp_err_t pwmmotor_set_speed_all(int speed_1, int speed_2, int speed_3, int speed_4);
esp_err_t pwmmotor_stop(motor_id_t motor_id, bool brake);

#ifdef __cplusplus
}
#endif
