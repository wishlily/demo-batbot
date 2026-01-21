#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MOTOR_MAX_SPEED (1.0f) ///< Maximum motor speed setting in meters per second

/**
 * @brief Initialize the motor driver system
 */
esp_err_t motor_init(void);

/**
 * @brief Set target speeds for all four motors
 *
 * @param speed_m1 Target speed for motor 1 in m/s
 * @param speed_m2 Target speed for motor 2 in m/s
 * @param speed_m3 Target speed for motor 3 in m/s
 * @param speed_m4 Target speed for motor 4 in m/s
 */
void motor_set_speed(float speed_m1, float speed_m2, float speed_m3, float speed_m4);

/**
 * @brief Get current speeds of all four motors
 *
 * @param[out] speed_m1 Current speed of motor 1 in m/s
 * @param[out] speed_m2 Current speed of motor 2 in m/s
 * @param[out] speed_m3 Current speed of motor 3 in m/s
 * @param[out] speed_m4 Current speed of motor 4 in m/s
 */
void motor_get_speed(float* speed_m1, float* speed_m2, float* speed_m3, float* speed_m4);

/**
 * @brief Stop all motors with optional braking
 *
 * @param brake true to actively brake, false to coast to stop
 */
esp_err_t motor_stop(bool brake);

/**
 * @brief Update PID controller parameters
 *
 * @param pid_p Proportional gain
 * @param pid_i Integral gain
 * @param pid_d Derivative gain
 */
esp_err_t motor_update_pid_parm(float pid_p, float pid_i, float pid_d, float pid_kf);

/**
 * @brief Read current PID controller parameters
 *
 * @param[out] out_p Pointer to store proportional gain
 * @param[out] out_i Pointer to store integral gain
 * @param[out] out_d Pointer to store derivative gain
 * @param[out] out_kf Pointer to store feed forward gain
 */
void motor_read_pid_parm(float* out_p, float* out_i, float* out_d, float* out_kf);

int motor_get_error_count(void);
void motor_calibration(void);

#ifdef __cplusplus
}
#endif
