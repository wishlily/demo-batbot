#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ms200.h"

typedef struct {
    uint64_t timestamp; // us, first point timestamp
    ms200_point_t points[MS200_POINT_MAX];
} ms200_frame_t;

int lidar_ms200_init(void);
int lidar_ms200_deinit(void);

int lidar_ms200_read(ms200_frame_t* frame, int32_t wait_ticks);

#ifdef __cplusplus
}
#endif
