#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"

#define ENCODER_GPIO_H1A 6
#define ENCODER_GPIO_H1B 7

#define ENCODER_GPIO_H2A 47
#define ENCODER_GPIO_H2B 48

#define ENCODER_GPIO_H3A 11
#define ENCODER_GPIO_H3B 12

#define ENCODER_GPIO_H4A 1
#define ENCODER_GPIO_H4B 2

#define ENCODER_PCNT_LIMIT 1000
#define ENCODER_PCNT_HIGH_LIMIT ENCODER_PCNT_LIMIT
#define ENCODER_PCNT_LOW_LIMIT -ENCODER_PCNT_LIMIT

typedef enum _encoder_id {
    ENCODER_ID_M1,
    ENCODER_ID_M2,
    ENCODER_ID_M3,
    ENCODER_ID_M4,
    ENCODER_ID_MAX,
} encoder_id_t;

esp_err_t encoder_init(void);
esp_err_t encoder_get_count(encoder_id_t encoder_id, int* count);

#ifdef __cplusplus
}
#endif
