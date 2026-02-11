#include "imu_fusion.h"

#include <string.h>

#define ADC_SCALE_16BIT 32768.0f

void imu_fusion_init(imu_fusion_handle_t* handle, float sample_rate, float gain, int fsr_g, int fsr_dps)
{
    memset(handle, 0, sizeof(imu_fusion_handle_t));
    FusionAhrsInitialise(&handle->ahrs);
    FusionAhrsSettings settings = {.convention = FusionConventionNwu,
                                   .gain = gain,
                                   .gyroscopeRange = (float)fsr_dps,
                                   .accelerationRejection = 10.0f,
                                   .magneticRejection = 0.0f,
                                   .recoveryTriggerPeriod = 5};
    FusionAhrsSetSettings(&handle->ahrs, &settings);
    FusionOffsetInitialise(&handle->offset, (int)sample_rate);
    handle->acc_scale = (float)fsr_g / ADC_SCALE_16BIT;
    handle->gyro_scale = (float)fsr_dps / ADC_SCALE_16BIT;
}

int imu_fusion_process_quaternion(imu_fusion_handle_t* handle,
                                  const int16_t* raw_accel,
                                  const int16_t* raw_gyro,
                                  imu_fusion_data_t* data_out,
                                  float dt)
{
    if (handle == NULL || raw_accel == NULL || raw_gyro == NULL || data_out == NULL)
        return -1;

    // Convert raw data to algorithm units (g, deg/s)
    FusionVector acc_g = {.axis.x = (float)raw_accel[0] * handle->acc_scale,
                          .axis.y = (float)raw_accel[1] * handle->acc_scale,
                          .axis.z = (float)raw_accel[2] * handle->acc_scale};
    FusionVector gyr_dps = {.axis.x = (float)raw_gyro[0] * handle->gyro_scale,
                            .axis.y = (float)raw_gyro[1] * handle->gyro_scale,
                            .axis.z = (float)raw_gyro[2] * handle->gyro_scale};

    // Gyro dynamic zero-bias compensation ---
    // The algorithm automatically learns the drift when stationary and subtracts it in real-time
    gyr_dps = FusionOffsetUpdate(&handle->offset, gyr_dps);

    // Update AHRS algorithm ---
    FusionAhrsUpdateNoMagnetometer(&handle->ahrs, gyr_dps, acc_g, dt);

    // Package output data ---
    // 1. Attitude output
    FusionQuaternion q = FusionAhrsGetQuaternion(&handle->ahrs);
    data_out->q[0] = q.element.w;
    data_out->q[1] = q.element.x;
    data_out->q[2] = q.element.y;
    data_out->q[3] = q.element.z;

    // Reuse the intermediate values already calculated and convert to SI units
    data_out->accel_mss[0] = acc_g.axis.x * IMU_G_TO_MSS;
    data_out->accel_mss[1] = acc_g.axis.y * IMU_G_TO_MSS;
    data_out->accel_mss[2] = acc_g.axis.z * IMU_G_TO_MSS;

    data_out->gyro_rads[0] = gyr_dps.axis.x * IMU_DPS_TO_RAD;
    data_out->gyro_rads[1] = gyr_dps.axis.y * IMU_DPS_TO_RAD;
    data_out->gyro_rads[2] = gyr_dps.axis.z * IMU_DPS_TO_RAD;

    return 0;
}