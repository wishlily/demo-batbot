#include "lidar_ms200.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "uart.h"

static const char* TAG = "LIDAR_MS200";

#define UART_PORT UART_NUM_1
#define UART_GPIO_TXD (GPIO_NUM_17)
#define UART_GPIO_RXD (GPIO_NUM_18)

static void uart_on_recv(const uint8_t* data, size_t len)
{
    for (int i = 0; i < len; i++) {
        ms200_data_receive(data[i]);
    }
}

#define LIDAR_DATA_QUEUE_SIZE 10

typedef struct {
    drv_uart_t udev;     // uart device
    ms200_frame_t buf;   // frame buffer
    uint16_t last_angle; // frame last save degree 0~MS200_POINT_MAX
    QueueHandle_t queue; // frame queue
} lidar_t;

static lidar_t lidar_dev;

#define MS200_ANGLE_UNIT 100 // 0.01 degree
#define MS200_FULL_ANGLE (360 * MS200_ANGLE_UNIT)
static void ms200_update_pkg(ms200_package_t* pkg)
{
    lidar_t* dev = &lidar_dev;

    float step_angle = 0.0f;
    float current_angle_f = 0.0f;
    uint16_t angle = 0;
    if (pkg->count <= 1) {
        ESP_LOGW(TAG, "Get Invalid count package");
        return;
    }

    uint16_t diff = 0;
    if (pkg->end_angle > pkg->start_angle) {
        // normal codition
        diff = pkg->end_angle - pkg->start_angle;
    } else {
        // Special case: The end Angle is smaller than the start Angle
        diff = MS200_FULL_ANGLE + pkg->end_angle - pkg->start_angle;
    }
    step_angle = (float)diff / (pkg->count - 1);
    current_angle_f = (float)pkg->start_angle;

    for (int i = 0; i < pkg->count; i++) {
        uint16_t raw_angle = (uint16_t)current_angle_f;
        angle = (raw_angle / MS200_ANGLE_UNIT) % MS200_POINT_MAX;
        current_angle_f += step_angle;

        if (pkg->points[i].intensity <= 15) { // invalid point
            continue;
        }
        if (angle < dev->last_angle) {     // new frame
            if (dev->buf.timestamp == 0) { // first frame
                dev->buf.timestamp = esp_timer_get_time();
            }
            if (xQueueSend(dev->queue, &dev->buf, 0) != pdTRUE) { // send old frame
                ESP_LOGW(TAG, "Queue full, packet dropped.");
            }
            memset(&dev->buf, 0, sizeof(ms200_frame_t));
            dev->buf.timestamp = esp_timer_get_time();
        }
        dev->buf.points[angle].distance = pkg->points[i].distance;
        dev->buf.points[angle].intensity = pkg->points[i].intensity;
        dev->last_angle = angle;
    }
}

int lidar_ms200_init(void)
{
    lidar_t* dev = &lidar_dev;
    memset(dev, 0, sizeof(lidar_t));
    dev->queue = xQueueCreate(LIDAR_DATA_QUEUE_SIZE, sizeof(ms200_frame_t));
    if (dev->queue == NULL) {
        ESP_LOGE(TAG, "Create lidar data queue failed");
        return ESP_ERR_NO_MEM;
    }
    ms200_set_update_cb(ms200_update_pkg);
    int rc = drv_uart_init(&dev->udev, UART_PORT, UART_GPIO_TXD, UART_GPIO_RXD, 230400, uart_on_recv);
    if (rc != UART_OK) {
        ESP_LOGE(TAG, "Init lidar uart failed %d", rc);
        lidar_ms200_deinit();
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

int lidar_ms200_deinit(void)
{
    lidar_t* dev = &lidar_dev;
    drv_uart_deinit(&dev->udev);
    vQueueDelete(dev->queue);
    dev->queue = NULL;
    return ESP_OK;
}

int lidar_ms200_read(ms200_frame_t* frame, int32_t wait_ticks)
{
    lidar_t* dev = &lidar_dev;
    if (xQueueReceive(dev->queue, frame, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}