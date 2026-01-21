#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// unit :m
#define ROBOT_WIDTH (0.135f)
#define ROBOT_LENGTH (0.095f)

typedef struct {
    float Vx;
    float Vy; // Vy ignored for differential drive.
    float Wz;
} motion_cmd_t;

esp_err_t motion_stop(bool brake);
esp_err_t motion_ctrl(motion_cmd_t* cmd);
esp_err_t motion_get_speed(motion_cmd_t* out);

esp_err_t motion_init(void);

#ifdef __cplusplus
}
#endif
