#include "motor.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pid_ctrl.h"
#include "pwm_motor.h"
#include "encoder.h"

static const char* TAG = "MOTOR";

#define MOTOR_MAX_NUM MOTOR_ID_MAX
#define MOTOR_WHEEL_PERIMETER_M (0.1508f)   // Wheel circumference m
#define MOTOR_ENCODER_TOTAL_PULSE (1040.0f) // Number of pulses generated per motor revolution: 13*20*4
#define MOTOR_PID_PERIOD_S (0.01f)          // PID algorithm calculation period: 10ms

// Speed filter coefficient (0.0~1.0)
// The smaller the smoother but the greater the lag
#define SPEED_FILTER_ALPHA (0.4f)


typedef struct {
    int last_pulse_count;

    float target_speed_m_s;
    float current_speed_m_s; // after filter
    pid_ctrl_block_handle_t pid_block;

    float pid_output;
    float pid_target;
} motor_t;

typedef struct {
    pid_ctrl_parameter_t pid_params;
    float kf; // Feed Forward Gain
    esp_timer_handle_t pid_timer_handle;

    uint32_t error_count;
    bool enabled; // Volatile as it's shared between Task and ISR

    motor_t motor[MOTOR_MAX_NUM];
} motor_ctrl_t;

static motor_ctrl_t motor_ctrl;

inline static float limit_value(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

// inline static float low_pass_filter(float current_val, float last_val)
// {
//     return last_val * (1.0f - SPEED_FILTER_ALPHA) + current_val * SPEED_FILTER_ALPHA;
// }

// Since esp_pid_ctrl doesn't allow clearing integral easily, we recreate the block.
static esp_err_t reset_pid_block(int index)
{
    if (motor_ctrl.motor[index].pid_block) {
        pid_del_control_block(motor_ctrl.motor[index].pid_block);
    }

    pid_ctrl_config_t pid_config = {
        .init_param = motor_ctrl.pid_params,
    };
    return pid_new_control_block(&pid_config, &motor_ctrl.motor[index].pid_block);
}

// Circular difference
inline static int calc_delta(int curr, int prev)
{
    int delta = curr - prev;

    if (delta > (ENCODER_PCNT_LIMIT + 1) / 2)
        delta -= ENCODER_PCNT_LIMIT;
    else if (delta < -(ENCODER_PCNT_LIMIT + 1) / 2)
        delta += ENCODER_PCNT_LIMIT;

    return delta;
}

// Pre-calculated conversion factor: (Perimeter / (Total_Pulse * Period))
static const float PULSE_TO_SPEED_FACTOR =
    MOTOR_WHEEL_PERIMETER_M / (MOTOR_ENCODER_TOTAL_PULSE * MOTOR_PID_PERIOD_S);

// Timer Callback (ISR Context)
static void motor_pid_timer_cb(void* arg)
{
    motor_ctrl_t* ctrl = &motor_ctrl;

    int cur_pulse_count;
    int delta_pulse;
    float pid_output;
    float final_pwm;

    for (int i = 0; i < MOTOR_MAX_NUM; i++) {
        // 1. Update Speed State (Must run even when disabled to keep delta valid)
        if (encoder_get_count(i, &cur_pulse_count) != ESP_OK) {
            ctrl->error_count++;
            continue; // Skip calculation if encoder fails
        }

        delta_pulse = calc_delta(cur_pulse_count, ctrl->motor[i].last_pulse_count);
        ctrl->motor[i].last_pulse_count = cur_pulse_count;

        ctrl->motor[i].current_speed_m_s = (float)delta_pulse * PULSE_TO_SPEED_FACTOR;

        // 2. Control Loop
        if (ctrl->enabled) {
            float error = ctrl->motor[i].target_speed_m_s - ctrl->motor[i].current_speed_m_s;
            if (pid_compute(ctrl->motor[i].pid_block, error, &pid_output) != ESP_OK) {
                ctrl->error_count++;
            }

            // Feed Forward
            float ff = ctrl->motor[i].target_speed_m_s * ctrl->kf;

            final_pwm = pid_output + ff;
            if (pwmmotor_set_speed(i, (int)final_pwm) != ESP_OK) {
                ctrl->error_count++;
            }
        }
        // If !enabled, we do NOTHING to the motor here.
        // This allows motor_stop()'s brake/coast setting to persist.
    }
}

esp_err_t motor_init(void)
{
    esp_err_t rc = encoder_init();
    if (rc != ESP_OK) {
        return rc;
    }
    rc = pwmmotor_init();
    if (rc != ESP_OK) {
        return rc;
    }

    motor_ctrl_t* ctrl = &motor_ctrl;
    memset(ctrl, 0, sizeof(motor_ctrl_t)); // Clear all states
    ctrl->enabled = false;

    // Default PID Parameters
    ctrl->pid_params.kp = 120.0f; // Reset to safe defaults or calibrated values
    ctrl->pid_params.ki = 6.0f;
    ctrl->pid_params.kd = 1.5f;
    ctrl->kf = 177.0f; // Adjusted KF based on PWM 200 -> 1.13 m/s

    ctrl->pid_params.cal_type = PID_CAL_TYPE_POSITIONAL;
    ctrl->pid_params.max_output = PWM_MOTOR_MAX_VALUE;
    ctrl->pid_params.min_output = -PWM_MOTOR_MAX_VALUE;
    ctrl->pid_params.max_integral = 200.f;
    ctrl->pid_params.min_integral = -200.f;
    pid_ctrl_config_t pid_config = {
        .init_param = ctrl->pid_params,
    };
    for (int i = 0; i < MOTOR_MAX_NUM; i++) {
        rc = pid_new_control_block(&pid_config, &ctrl->motor[i].pid_block);
        if (rc != ESP_OK) {
            return rc;
        }
        // Initialize last_pulse to current to avoid startup spike
        rc = encoder_get_count(i, &ctrl->motor[i].last_pulse_count);
        if (rc != ESP_OK) {
            return rc;
        }
    }

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &motor_pid_timer_cb,
        .name = "pid_loop",
        .dispatch_method = ESP_TIMER_TASK, // Or ESP_TIMER_ISR if your code is very short/fast
        .skip_unhandled_events = true,
    };

    rc = esp_timer_create(&periodic_timer_args, &ctrl->pid_timer_handle);
    if (rc != ESP_OK) {
        return rc;
    }
    rc = esp_timer_start_periodic(ctrl->pid_timer_handle, (uint64_t)(MOTOR_PID_PERIOD_S * 1000000));
    if (rc != ESP_OK) {
        return rc;
    }
    ESP_LOGI(TAG, "Motor Control Initialized");
    return rc;
}

void motor_set_speed(float speed_m1, float speed_m2, float speed_m3, float speed_m4)
{
    motor_ctrl_t* ctrl = &motor_ctrl;
    // If we are transitioning from Stopped -> Running, reset PID to clear old errors
    if (!ctrl->enabled) {
        for (int i = 0; i < MOTOR_MAX_NUM; i++) {
            // Optional: Reset PID state here if you want a clean start every time
            // reset_pid_block(i);
        }
    }
    // XXX: Race Condition
    ctrl->motor[MOTOR_ID_M1].target_speed_m_s = limit_value(speed_m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    ctrl->motor[MOTOR_ID_M2].target_speed_m_s = limit_value(speed_m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    ctrl->motor[MOTOR_ID_M3].target_speed_m_s = limit_value(speed_m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    ctrl->motor[MOTOR_ID_M4].target_speed_m_s = limit_value(speed_m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    // Re-enable control loop
    ctrl->enabled = true;
}

void motor_get_speed(float* speed_m1, float* speed_m2, float* speed_m3, float* speed_m4)
{
    motor_ctrl_t* ctrl = &motor_ctrl;
    *speed_m1 = ctrl->motor[MOTOR_ID_M1].current_speed_m_s;
    *speed_m2 = ctrl->motor[MOTOR_ID_M2].current_speed_m_s;
    *speed_m3 = ctrl->motor[MOTOR_ID_M3].current_speed_m_s;
    *speed_m4 = ctrl->motor[MOTOR_ID_M4].current_speed_m_s;
}

esp_err_t motor_stop(bool brake)
{
    motor_ctrl_t* ctrl = &motor_ctrl;
    ctrl->enabled = false;

    for (int i = 0; i < MOTOR_MAX_NUM; i++) {
        ctrl->motor[i].target_speed_m_s = 0.0f;
    }

    for (int i = 0; i < MOTOR_MAX_NUM; i++) {
        reset_pid_block(i);
    }

    return pwmmotor_stop(MOTOR_ID_ALL, brake);
}

esp_err_t motor_update_pid_parm(float pid_p, float pid_i, float pid_d, float pid_kf)
{
    motor_ctrl_t* ctrl = &motor_ctrl;

    ctrl->pid_params.kp = pid_p;
    ctrl->pid_params.ki = pid_i;
    ctrl->pid_params.kd = pid_d;
    ctrl->kf = pid_kf;

    for (int i = 0; i < MOTOR_MAX_NUM; i++) {
        int rc = pid_update_parameters(ctrl->motor[i].pid_block, &ctrl->pid_params);
        if (rc != ESP_OK) {
            return rc;
        }
    }
    return ESP_OK;
}

void motor_read_pid_parm(float* out_p, float* out_i, float* out_d, float* out_kf)
{
    motor_ctrl_t* ctrl = &motor_ctrl;

    *out_p = ctrl->pid_params.kp;
    *out_i = ctrl->pid_params.ki;
    *out_d = ctrl->pid_params.kd;
    *out_kf = ctrl->kf;
}

int motor_get_error_count(void)
{
    motor_ctrl_t* ctrl = &motor_ctrl;
    return ctrl->error_count;
}

void motor_calibration(void)
{
    ESP_LOGI(TAG, "Starting Calibration...");
    motor_stop(false); // Ensure PID control is disabled
    vTaskDelay(pdMS_TO_TICKS(1000));

    int pwm_steps = 20;
    for (int pwm = 160; pwm <= PWM_MOTOR_DUTY_TICK_MAX / 2; pwm += pwm_steps) {
        ESP_LOGI(TAG, "Testing PWM: %d", pwm);

        // Set raw PWM directly
        pwmmotor_set_speed(MOTOR_ID_ALL, pwm);

        for (int i = 0; i < 3; i++) {
            // Wait for speed to stabilize
            vTaskDelay(pdMS_TO_TICKS(1000));

            // Read speed
            float s1, s2, s3, s4;
            motor_get_speed(&s1, &s2, &s3, &s4);
            float avg_speed = (s1 + s2 + s3 + s4) / 4.0f;

            ESP_LOGI(TAG,
                     "PWM: %d, Speed: %.4f|%.4f|%.4f|%.4f|%.4f, Ratio(PWM/Speed): %.2f, err: %d",
                     pwm,
                     avg_speed,
                     s1,
                     s2,
                     s3,
                     s4,
                     (avg_speed > 0.001) ? (pwm / avg_speed) : 0,
                     motor_get_error_count());
        }
    }

    pwmmotor_stop(MOTOR_ID_ALL, true);
    ESP_LOGI(TAG, "Calibration Complete.");
}