#include "uart.h"

#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include "esp_log.h"
#include "esp_system.h"

#define UART_EVENT_QUEUE_SIZE 64
#define TEMP_BUF_SIZE 512
#define UART_DRIVER_RX_BUF_SIZE 4096
static void drv_uart_rx_task(void* arg)
{
    uart_event_t event;
    drv_uart_t* dev = (drv_uart_t*)arg;
    ESP_LOGI(dev->tag, "Task Started on Core %d", xPortGetCoreID());

    uint8_t dtmp[TEMP_BUF_SIZE];
    while (1) {
        if (xQueueReceive(dev->event_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA: {
                size_t buffered_size;
                uart_get_buffered_data_len(dev->port, &buffered_size);

                while (buffered_size > 0) {
                    size_t length_to_read = MIN(TEMP_BUF_SIZE, buffered_size);
                    int len = uart_read_bytes(dev->port, dtmp, length_to_read, 0);

                    if (len > 0 && dev->on_recv_fn) {
                        dev->on_recv_fn(dtmp, len);
                    }
                    uart_get_buffered_data_len(dev->port, &buffered_size);
                }
                break;
            }
            case UART_FIFO_OVF:
                ESP_LOGW(dev->tag, "HW FIFO Overflow");
                uart_flush_input(dev->port);
                xQueueReset(dev->event_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(dev->tag, "Ring Buffer Full");
                uart_flush_input(dev->port);
                xQueueReset(dev->event_queue);
                break;
            default:
                ESP_LOGI(dev->tag, "uart event type: %d", event.type);
                break;
            }
        }
    }

    vTaskDelete(NULL);
}

drv_uart_error_t
drv_uart_init(drv_uart_t* dev, uart_port_t port, int tx, int rx, int baud, drv_uart_on_receive_t cb)
{
    // defensive programming
    if (dev == NULL) {
        return UART_ERROR_PARAM;
    }

    memset(dev, 0, sizeof(drv_uart_t));

    dev->port = port;
    dev->tx_pin = tx;
    dev->rx_pin = rx;
    dev->baud_rate = baud;
    dev->on_recv_fn = cb;

    if (port == UART_NUM_0)
        dev->tag = "UART0";
    else if (port == UART_NUM_1)
        dev->tag = "UART1";
    else
        dev->tag = "UARTX";

    const uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int rc = uart_driver_install(
        port, UART_DRIVER_RX_BUF_SIZE, 0, UART_EVENT_QUEUE_SIZE, &dev->event_queue, ESP_INTR_FLAG_IRAM);
    if (rc != ESP_OK) {
        goto deinit;
    }
    rc = uart_param_config(port, &uart_config);
    if (rc != ESP_OK) {
        goto deinit;
    }
    rc = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (rc != ESP_OK) {
        goto deinit;
    }

    char task_name[16];
    snprintf(task_name, sizeof(task_name), "uart%d_task", port);

    BaseType_t ret = xTaskCreate(
        drv_uart_rx_task, task_name, 4096, (void*)dev, configMAX_PRIORITIES - 3, &dev->task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(dev->tag, "Task Create Fail");
        goto deinit;
    }
    ESP_LOGI(dev->tag, "Init Success");
    return UART_OK;

deinit:
    drv_uart_deinit(dev);
    return UART_ERROR_INIT;
}

// normal not use
void drv_uart_deinit(drv_uart_t* dev)
{
    if (dev == NULL)
        return;
    if (dev->task_handle != NULL) {
        vTaskDelete(dev->task_handle);
        dev->task_handle = NULL;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (uart_is_driver_installed(dev->port)) {
        uart_driver_delete(dev->port);
    }
    ESP_LOGI(dev->tag, "UART De-initialized");
}

int drv_uart_send(drv_uart_t* dev, uint8_t* data, uint16_t len)
{
    if (dev == NULL || data == NULL) {
        return -1;
    }
    return uart_write_bytes(dev->port, (const char*)data, len);
}