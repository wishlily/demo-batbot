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

#ifndef _SYSTEM_INTERFACE_H_
#define _SYSTEM_INTERFACE_H_

#include "imu/inv_imu_transport.h"

#define IMU_LOG_TAG "INV_IMU"
#define IMU_WRITE_STACK_BUF_SIZE 8

#define I2C_MASTER_GPIO_SCL 39
#define I2C_MASTER_GPIO_SDA 40
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_PORT_NUM 0

#define I2C_SLAVE_ADDR 0x68
#define I2C_SLAVE_ADDR_BIT_LEN I2C_ADDR_BIT_LEN_7

#define I2C_MASTER_TIMEOUT_MS 1000

/*
 * I/O for IMU device
 */

/** @brief Configure I/O for IMU device.
 *  @param[in] serif_type  Serial interface type to be used.
 *  @return                0 on success, negative value on error.
 */
int si_io_imu_init(uint32_t serif_type);
void* si_io_imu_context(void);

/** @brief Read register(s) implementation for IMU device.
 *  @param[in] serif  Serial interface.
 *  @param[in] reg   Register address to be read.
 *  @param[out] buf  Output data from the register.
 *  @param[in] len   Number of byte to be read.
 *  @return          0 on success, negative value on error.
 */
int si_io_imu_read_reg(struct inv_imu_serif* serif, uint8_t reg, uint8_t* buf, uint32_t len);

/** @brief Write register(s) implementation for IMU device.
 *  @param[in] serif  Serial interface.
 *  @param[in] reg    Register address to be written.
 *  @param[in] buf    Input data to write.
 *  @param[in] len    Number of byte to be written.
 *  @return           0 on success, negative value on error.
 */
int si_io_imu_write_reg(struct inv_imu_serif* serif, uint8_t reg, const uint8_t* buf, uint32_t len);

/*
 * Timers
 */

/** @brief Sleep function implementation.
 *  @param[in] us  Time to sleep in microseconds.
 */
void si_sleep_us(uint32_t us);

/** @brief Get current time function implementation.
 *  @return  The current time in us or 0 if get_time_us pointer is null.
 */
uint64_t si_get_time_us();

/*
 * GPIO
 */

#define SI_GPIO_INT1 41

/** @brief Initializes GPIO module to trigger callback when interrupt from IMU fires.
 *  @param[in] int_num  Interrupt pin (`SI_GPIO_INT1` or `SI_GPIO_INT2`).
 *  @param[in] int_cb   Callback to be called when interrupt fires.
 *  @return             0 on success, negative value on error.
 */
int si_init_gpio_int(unsigned int_num, void (*int_cb)(void* args));

/*
 * Common
 */

/** @brief Disable core interrupts.
 */
void si_disable_irq();

/** @brief Enable core interrupts.
 */
void si_enable_irq();

#endif /* !_SYSTEM_INTERFACE_H_ */
