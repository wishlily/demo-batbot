/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "system_interface.h"

/* Standard includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* IMU drivers */
#include "imu/inv_imu_defs.h" /* For error codes */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"

#ifndef IMU_WRITE_STACK_BUF_SIZE
#define IMU_WRITE_STACK_BUF_SIZE 32
#endif

// XXX: I2C bus only use in IMU driver
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

static void si_io_imu_deinit_device(void)
{
    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }
    if (bus_handle) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
    }
}

int si_io_imu_init(uint32_t serif_type)
{
    if (serif_type != UI_I2C) {
        return INV_ERROR_BAD_ARG;
    }
    si_io_imu_deinit_device();

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_MASTER_GPIO_SCL,
        .sda_io_num = I2C_MASTER_GPIO_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(IMU_LOG_TAG, "I2C Bus Init Failed: %s", esp_err_to_name(err));
        return INV_ERROR_TRANSPORT;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_SLAVE_ADDR_BIT_LEN,
        .device_address = I2C_SLAVE_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(IMU_LOG_TAG, "I2C Device Add Failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        return INV_ERROR_TRANSPORT;
    }

    ESP_LOGI(IMU_LOG_TAG, "I2C initialized successfully");
    return 0;
}

void* si_io_imu_context(void)
{
    return (void*)dev_handle;
}

int si_io_imu_read_reg(struct inv_imu_serif* serif, uint8_t reg, uint8_t* buf, uint32_t len)
{
    i2c_master_dev_handle_t handle = (i2c_master_dev_handle_t)serif->context;
    if (handle == NULL) {
        return INV_ERROR_BAD_ARG;
    }

    esp_err_t ret =
        i2c_master_transmit_receive(handle, &reg, 1, buf, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    // ESP_LOG_BUFFER_HEX("I2C_RX", buf, len);
    return (ret == ESP_OK) ? INV_ERROR_SUCCESS : INV_ERROR_IO;
}

int si_io_imu_write_reg(struct inv_imu_serif* serif, uint8_t reg, const uint8_t* buf, uint32_t len)
{
    i2c_master_dev_handle_t handle = (i2c_master_dev_handle_t)serif->context;
    if (handle == NULL) {
        return INV_ERROR_BAD_ARG;
    }

    esp_err_t ret;
    uint8_t stack_buf[IMU_WRITE_STACK_BUF_SIZE];
    uint8_t* tx_buf = NULL;
    size_t total_len = len + 1; // reg(1) + buf(len)

    if (total_len <= IMU_WRITE_STACK_BUF_SIZE) {
        tx_buf = stack_buf;
    } else {
        tx_buf = (uint8_t*)malloc(total_len);
        if (tx_buf == NULL) {
            return INV_ERROR_MEM;
        }
    }

    tx_buf[0] = reg;
    if (len > 0 && buf != NULL) {
        memcpy(&tx_buf[1], buf, len);
    }

    ret = i2c_master_transmit(handle, tx_buf, total_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    // ESP_LOG_BUFFER_HEX("I2C_TX", tx_buf, total_len);

    if (tx_buf != stack_buf) {
        free(tx_buf);
    }
    return (ret == ESP_OK) ? INV_ERROR_SUCCESS : INV_ERROR_IO;
}

/*
 * Timers
 */
void si_sleep_us(uint32_t us)
{
    esp_rom_delay_us(us); // blocking
}

uint64_t si_get_time_us()
{
    return (uint64_t)esp_timer_get_time();
}

/*
 * GPIO
 */
int si_init_gpio_int(unsigned int_num, void (*int_cb)(void* args))
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << int_num);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return INV_ERROR_BAD_ARG;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(IMU_LOG_TAG, "ISR install failed: %s", esp_err_to_name(err));
        return INV_ERROR_UNEXPECTED;
    }

    err = gpio_isr_handler_add(int_num, int_cb, (void*)int_num);
    if (err != ESP_OK) {
        return INV_ERROR_UNEXPECTED;
    }
    return 0;
}

/*
 * Common
 */
static portMUX_TYPE si_spinlock = portMUX_INITIALIZER_UNLOCKED;

void si_disable_irq()
{
    portENTER_CRITICAL(&si_spinlock);
}

void si_enable_irq()
{
    portEXIT_CRITICAL(&si_spinlock);
}
