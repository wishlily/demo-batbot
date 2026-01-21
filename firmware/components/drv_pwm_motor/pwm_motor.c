#include "pwm_motor.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "bdc_motor.h"


static const char* TAG = "PWM_MOTOR";

static bdc_motor_handle_t g_motors[MOTOR_ID_MAX];

#define CHECK_RETURN_IF_ERROR(x)                                \
    do {                                                        \
        esp_err_t err = (x);                                    \
        if (err != ESP_OK) {                                    \
            ESP_LOGE(TAG, "Error (%s:%d)", __FILE__, __LINE__); \
            return err;                                         \
        }                                                       \
    } while (0)

// Motor dead zone filtering
static int apply_dead_zone(int speed)
{
    if (speed > 0)
        return speed + PWM_MOTOR_DEAD_ZONE;
    if (speed < 0)
        return speed - PWM_MOTOR_DEAD_ZONE;
    return 0;
}

// Limits the maximum and minimum input values.
static int limit_speed(int speed)
{
    if (speed > PWM_MOTOR_DUTY_TICK_MAX)
        return PWM_MOTOR_DUTY_TICK_MAX;
    if (speed < -PWM_MOTOR_DUTY_TICK_MAX)
        return -PWM_MOTOR_DUTY_TICK_MAX;
    return speed;
}

inline static esp_err_t set_single_motor_stop(int index, bool brake)
{
    if (brake)
        return bdc_motor_brake(g_motors[index]);
    CHECK_RETURN_IF_ERROR(bdc_motor_coast(g_motors[index]));
    return ESP_OK;
}

static esp_err_t set_single_motor_speed(int index, int speed)
{
    int final_speed = limit_speed(apply_dead_zone(speed));

    if (final_speed > 0) {
        CHECK_RETURN_IF_ERROR(bdc_motor_forward(g_motors[index]));
        CHECK_RETURN_IF_ERROR(bdc_motor_set_speed(g_motors[index], final_speed));
    } else if (final_speed < 0) {
        CHECK_RETURN_IF_ERROR(bdc_motor_reverse(g_motors[index]));
        CHECK_RETURN_IF_ERROR(bdc_motor_set_speed(g_motors[index], -final_speed));
    } else {
        CHECK_RETURN_IF_ERROR(set_single_motor_stop(index, false));
    }
    return ESP_OK;
}

// Initial motor
esp_err_t pwmmotor_init(void)
{
    ESP_LOGI(TAG, "Init PwmMotor Device");

    struct motor_conf {
        int pwma;
        int pwmb;
        int group_id;
    } conf[MOTOR_ID_MAX] = {
        {PWM_GPIO_M1B, PWM_GPIO_M1A, PWM_MOTOR_TIMER_GROUP_ID_M1},
        {PWM_GPIO_M2B, PWM_GPIO_M2A, PWM_MOTOR_TIMER_GROUP_ID_M2},
        {PWM_GPIO_M3A, PWM_GPIO_M3B, PWM_MOTOR_TIMER_GROUP_ID_M3},
        {PWM_GPIO_M4A, PWM_GPIO_M4B, PWM_MOTOR_TIMER_GROUP_ID_M4},
    };

    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        bdc_motor_config_t motor_config = {
            .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
            .pwma_gpio_num = conf[i].pwma,
            .pwmb_gpio_num = conf[i].pwmb,
        };
        bdc_motor_mcpwm_config_t mcpwm_config = {
            .group_id = conf[i].group_id,
            .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
        };
        esp_err_t ret = bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &g_motors[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create motor %d", i + 1);
            return ret;
        }
        ret = bdc_motor_enable(g_motors[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable motor %d", i + 1);
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t pwmmotor_set_speed(motor_id_t motor_id, int speed)
{
    if (motor_id < MOTOR_ID_MAX) {
        return set_single_motor_speed(motor_id, speed);
    }
    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        CHECK_RETURN_IF_ERROR(set_single_motor_speed(i, speed));
    }
    return ESP_OK;
}

esp_err_t pwmmotor_set_speed_all(int speed_1, int speed_2, int speed_3, int speed_4)
{
    int speed[MOTOR_ID_MAX] = {
        speed_1,
        speed_2,
        speed_3,
        speed_4,
    };
    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        CHECK_RETURN_IF_ERROR(set_single_motor_speed(i, speed[i]));
    }
    return ESP_OK;
}

esp_err_t pwmmotor_stop(motor_id_t motor_id, bool brake)
{
    if (motor_id < MOTOR_ID_MAX) {
        return set_single_motor_stop(motor_id, brake);
    }
    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        CHECK_RETURN_IF_ERROR(set_single_motor_stop(i, brake));
    }
    return ESP_OK;
}