#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define IMU_FIFO_SIZE 32

#define IMU_ACCEL_FSR_G 4
#define IMU_GYRO_FSR_DPS 2000

#define IMU_ACCEL_FSR_REG ACCEL_CONFIG0_FS_SEL_4g
#define IMU_GYRO_FSR_REG GYRO_CONFIG0_FS_SEL_2000dps

typedef struct {
    int accel_raw[3];
    int gyro_raw[3];
    uint64_t timestamp; // fifo timestamp
} imu_raw_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint64_t timestamp; // us
} imu_data_t;


int imu_init(void);
float imu_get_temperature(void);
int imu_wait_for_data(imu_data_t* data, int32_t ticks_to_wait);

#ifdef __cplusplus
}
#endif