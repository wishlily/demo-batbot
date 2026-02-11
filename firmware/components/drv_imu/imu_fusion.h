#pragma once

#include "Fusion.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_G_TO_MSS 9.80665f
#define IMU_DPS_TO_RAD (0.01745329f) // PI/180

typedef struct {
    FusionAhrs ahrs;
    FusionOffset offset;
    float acc_scale;  // Raw -> G
    float gyro_scale; // Raw -> DPS
} imu_fusion_handle_t;

typedef struct {
    float roll, pitch, yaw; // Euler angles (deg)
    float q[4];             // Quaternion [w, x, y, z]
    float accel_mss[3];     // Acceleration (m/s^2)
    float gyro_rads[3];     // Angular velocity (rad/s) - Bias removed
} imu_fusion_data_t;


/**
 * @brief Initialize the middleware
 * @param handle Pointer to handle
 * @param sample_rate Sample rate (Hz)
 * @param gain Algorithm gain (Madgwick beta, suggested 0.5)
 * @param fsr_g Full scale range of accelerometer (e.g. 2, 4, 8)
 * @param fsr_dps Full scale range of gyroscope (e.g. 2000)
 */
void imu_fusion_init(imu_fusion_handle_t* handle, float sample_rate, float gain, int fsr_g, int fsr_dps);

/**
 * @brief Process quaternion fusion algorithm with raw accelerometer and gyroscope data
 *
 * This function takes raw accelerometer and gyroscope data and performs quaternion-based
 * sensor fusion to calculate orientation.
 *
 * @param handle Handle to the IMU fusion instance
 * @param raw_accel Raw accelerometer data in counts (typically 16-bit signed integers)
 * @param raw_gyro Raw gyroscope data in counts (typically 16-bit signed integers)
 * @param data_out Output structure containing quaternion data
 * @param dt Time delta since last update in seconds, used for integration calculations
 * @return int Status code indicating success or failure of the fusion process
 */
int imu_fusion_process_quaternion(imu_fusion_handle_t* handle,
                                  const int16_t* raw_accel,
                                  const int16_t* raw_gyro,
                                  imu_fusion_data_t* data_out,
                                  float dt);

#ifdef __cplusplus
}
#endif