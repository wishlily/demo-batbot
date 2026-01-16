#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"

#define MS200_HEAD_1 0xAA
#define MS200_HEAD_2 0x55
#define MS200_TAIL_1 0x31
#define MS200_TAIL_2 0xF2
#define MS200_FLAG_SN 0x01
#define MS200_FLAG_VERSION 0x02

#define MS200_DATA_START 0x54

#define MS200_POINT_MAX 360
#define MS200_BUF_MAX 100

typedef enum {
    MS200_OK = 0,
    MS200_ERROR_TAIL = -1001, // parse tail error
    MS200_ERROR_CRC,
    MS200_ERROR_LEN
} ms200_error_t;

typedef struct {
    uint16_t distance; // mm
    uint8_t intensity; // 0~255
} __attribute__((packed)) ms200_point_t;

#define MS200_POINT_PER_PACK 12
typedef struct {
    uint8_t count;
    uint16_t speed;
    uint16_t start_angle;
    ms200_point_t points[MS200_POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
} ms200_package_t;

typedef void (*ms200_update_t)(ms200_package_t* pkg);

void ms200_set_update_cb(ms200_update_t cb);
void ms200_data_receive(uint8_t rxtemp);

#ifdef __cplusplus
}
#endif
