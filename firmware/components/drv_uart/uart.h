#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART1_GPIO_TXD (GPIO_NUM_17)
#define UART1_GPIO_RXD (GPIO_NUM_18)

typedef void (*drv_uart_on_receive_t)(const uint8_t* data, size_t len);

typedef struct {
    uart_port_t port; // UART_NUM_0, UART_NUM_1
    int tx_pin;
    int rx_pin;
    int baud_rate;

    QueueHandle_t event_queue;
    TaskHandle_t task_handle;

    drv_uart_on_receive_t on_recv_fn;
    const char* tag; // Log tag
} drv_uart_t;

typedef enum {
    UART_OK = 0,
    UART_ERROR_PARAM = -1001,
    UART_ERROR_BUFFER,
    UART_ERROR_INIT,
} drv_uart_error_t;

/**
 * @brief Initialize the UART driver instance.
 *
 * This function configures the ESP32 UART hardware, allocates the internal
 * StreamBuffer, and starts a high-priority FreeRTOS task for receiving data.
 *
 * @note **Memory Requirement**:
 *       The `dev` pointer must point to a valid memory region (e.g., a global variable
 *       or a heap-allocated struct). This function will initialize the internal members
 *       of the struct.
 *
 */
drv_uart_error_t
drv_uart_init(drv_uart_t* dev, uart_port_t port, int tx, int rx, int baud, drv_uart_on_receive_t cb);
void drv_uart_deinit(drv_uart_t* dev);
int drv_uart_send(drv_uart_t* dev, uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif
