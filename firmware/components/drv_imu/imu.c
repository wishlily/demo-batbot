#include "imu.h"

#include <stdatomic.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_selftest.h"

#include "system_interface.h"

#define IMU_ERROR_CHECK(x)                         \
    ({                                             \
        int rc = ESP_ERROR_CHECK_WITHOUT_ABORT(x); \
        if (rc != ESP_OK) {                        \
            return rc;                             \
        }                                          \
    })

static TaskHandle_t imu_task_handle = NULL;
static volatile uint64_t int1_timestamp;

static inv_imu_device_t imu_dev;
static QueueHandle_t imu_raw_queue = NULL;
static atomic_int imu_temperature_raw = ATOMIC_VAR_INIT(0);

/* Get time implementation for IMU driver */
uint64_t inv_imu_get_time_us(void)
{
    return si_get_time_us();
}

/* Sleep implementation for IMU driver */
void inv_imu_sleep_us(uint32_t us)
{
    si_sleep_us(us);
}

static void IRAM_ATTR gpio_isr_handler(void* args)
{
    int1_timestamp = (uint64_t)esp_timer_get_time();

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (imu_task_handle != NULL) {
        vTaskNotifyGiveFromISR(imu_task_handle, &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static int setup_mcu(void)
{
    IMU_ERROR_CHECK(si_init_gpio_int(SI_GPIO_INT1, gpio_isr_handler));
    IMU_ERROR_CHECK(si_io_imu_init(UI_I2C));
    return 0;
}

static int configure_fifo(void);
static int configure_hires(void);
static int configure_power_mode(void);
static void sensor_event_cb(inv_imu_sensor_event_t* event);
static int imu_selftest(void);

static int setup_imu(void)
{
    inv_imu_serif_t imu_serif;
    imu_serif.context = si_io_imu_context();
    imu_serif.read_reg = si_io_imu_read_reg;
    imu_serif.write_reg = si_io_imu_write_reg;
    imu_serif.max_read = 1024;  /* maximum number of bytes allowed per serial read */
    imu_serif.max_write = 1024; /* maximum number of bytes allowed per serial write */
    imu_serif.serif_type = UI_I2C;
    IMU_ERROR_CHECK(inv_imu_init(&imu_dev, &imu_serif, sensor_event_cb));

    IMU_ERROR_CHECK(imu_selftest());

    /*
     * Configure interrupts pins
     * - Polarity High
     * - Pulse mode
     * - Push-Pull drive
     */
    inv_imu_int1_pin_config_t int1_pin_config;
    int1_pin_config.int_polarity = INT_CONFIG_INT1_POLARITY_HIGH;
    int1_pin_config.int_mode = INT_CONFIG_INT1_MODE_PULSED;
    int1_pin_config.int_drive = INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
    IMU_ERROR_CHECK(inv_imu_set_pin_config_int1(&imu_dev, &int1_pin_config));

    /* Configure FSR (doesn't apply if FIFO is used in highres mode) */
    IMU_ERROR_CHECK(inv_imu_set_accel_fsr(&imu_dev, IMU_ACCEL_FSR_REG));
    IMU_ERROR_CHECK(inv_imu_set_gyro_fsr(&imu_dev, IMU_GYRO_FSR_REG));

    /* Configure ODR */
    IMU_ERROR_CHECK(inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ODR_50_HZ));
    IMU_ERROR_CHECK(inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_ODR_50_HZ));

    /* Variable configuration */
    IMU_ERROR_CHECK(configure_fifo());
    IMU_ERROR_CHECK(configure_hires());
    IMU_ERROR_CHECK(configure_power_mode());
    return 0;
}

static int configure_fifo()
{
    int rc = 0;
    inv_imu_interrupt_parameter_t int1_config = {(inv_imu_interrupt_value)0};

    rc |= inv_imu_configure_fifo(&imu_dev, INV_IMU_FIFO_ENABLED);

    /* Configure interrupts sources */
    int1_config.INV_FIFO_THS = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int1(&imu_dev, &int1_config);

    return rc;
}

static int configure_hires()
{
    int rc = 0;
    rc |= inv_imu_disable_high_resolution_fifo(&imu_dev);
    return rc;
}

static int configure_power_mode()
{
    int rc = 0;
    rc |= inv_imu_enable_accel_low_noise_mode(&imu_dev);
    rc |= inv_imu_enable_gyro_low_noise_mode(&imu_dev);
    return rc;
}

static void sensor_event_cb(inv_imu_sensor_event_t* event)
{
    uint64_t int_timestamp = 0;
    imu_raw_t raw;

    si_disable_irq();
    int_timestamp = int1_timestamp;
    si_enable_irq();

    static uint64_t last_fifo_timestamp = 0;
    static uint32_t rollover_num = 0;

    /* Handle rollover */
    if (last_fifo_timestamp > event->timestamp_fsync)
        rollover_num++;
    last_fifo_timestamp = event->timestamp_fsync;

    /* Compute timestamp in us */
    if (last_fifo_timestamp == 0 && rollover_num == 0) {
        raw.timestamp = int_timestamp;
    } else {
        raw.timestamp = event->timestamp_fsync + rollover_num * UINT16_MAX;
        raw.timestamp *= inv_imu_get_timestamp_resolution_us(&imu_dev);
    }
    raw.accel_raw[0] = event->accel[0];
    raw.accel_raw[1] = event->accel[1];
    raw.accel_raw[2] = event->accel[2];
    raw.gyro_raw[0] = event->gyro[0];
    raw.gyro_raw[1] = event->gyro[1];
    raw.gyro_raw[2] = event->gyro[2];

    BaseType_t ret = xQueueSend(imu_raw_queue, &raw, 0);
    if (ret != pdTRUE) {
        ESP_LOGW(IMU_LOG_TAG, "Queue full, packet dropped");
    }
    atomic_store(&imu_temperature_raw, event->temperature);
}

static int imu_selftest(void)
{
    ESP_LOGI(IMU_LOG_TAG, "Starting IMU selftest...");

    si_sleep_us(100000); // wait 100ms

    inv_imu_selftest_output_t out;
    inv_imu_selftest_parameters_t params;

    IMU_ERROR_CHECK(inv_imu_init_selftest_parameters_struct(&imu_dev, &params));

    int rc = inv_imu_run_selftest(&imu_dev, params, &out);
    if (rc != 0) {
        if (rc < 0) {
            ESP_LOGE(IMU_LOG_TAG, "Self-test IO/Driver Error: %d", rc);
        } else {
            ESP_LOGE(IMU_LOG_TAG, "Self-test Hardware Failure. Status Bitmap: 0x%x", rc);
        }
    } else {
        ESP_LOGI(IMU_LOG_TAG, "Self-test Result: PASS");
    }

    ESP_LOGI(IMU_LOG_TAG, "Accel self-test %s", out.accel_status == 1 ? "OK" : "KO");
    if (out.accel_status != 1) {
        ESP_LOGD(IMU_LOG_TAG, "  - Accel X: %s", out.ax_status == 1 ? "OK" : "KO");
        ESP_LOGD(IMU_LOG_TAG, "  - Accel Y: %s", out.ay_status == 1 ? "OK" : "KO");
        ESP_LOGD(IMU_LOG_TAG, "  - Accel Z: %s", out.az_status == 1 ? "OK" : "KO");
        rc |= INV_ERROR;
    }

#if INV_IMU_IS_GYRO_SUPPORTED
    ESP_LOGI(IMU_LOG_TAG, "Gyro self-test %s", out.gyro_status == 1 ? "OK" : "KO");
    if (out.gyro_status != 1) {
        ESP_LOGD(IMU_LOG_TAG, "  - Gyro X: %s", out.gx_status == 1 ? "OK" : "KO");
        ESP_LOGD(IMU_LOG_TAG, "  - Gyro Y: %s", out.gy_status == 1 ? "OK" : "KO");
        ESP_LOGD(IMU_LOG_TAG, "  - Gyro Z: %s", out.gz_status == 1 ? "OK" : "KO");
        rc |= INV_ERROR;
    }
#endif

    return rc;
}

static void imu_task(void* arg)
{
    ESP_LOGI(IMU_LOG_TAG, "IMU task started");
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
            inv_imu_get_data_from_fifo(&imu_dev);
        }
    }
    vTaskDelete(NULL);
}

int imu_init(void)
{
    imu_raw_queue = xQueueCreate(IMU_FIFO_SIZE, sizeof(imu_raw_t));
    if (imu_raw_queue == NULL) {
        return INV_ERROR_MEM;
    }

    IMU_ERROR_CHECK(setup_mcu());
    si_sleep_us(50000);
    IMU_ERROR_CHECK(setup_imu());

    xTaskCreate(imu_task, "imu_task", 4096, NULL, configMAX_PRIORITIES - 1, &imu_task_handle);
    return 0;
}

float imu_get_temperature(void)
{
    int temp = atomic_load(&imu_temperature_raw);
    return (25 + (float)temp / 2);
}

#define GRAVITY_MSS 9.80665f
#define ADC_SCALE_16BIT 32768.0f
#define DEG_TO_RAD (M_PI / 180.0f)

int imu_wait_for_data(imu_data_t* data, int32_t ticks_to_wait)
{
    imu_raw_t rx_data;
    if (xQueueReceive(imu_raw_queue, &rx_data, ticks_to_wait) == pdTRUE) {
        float accel_scale = (float)IMU_ACCEL_FSR_G * GRAVITY_MSS / ADC_SCALE_16BIT;
        float gyro_scale = (float)IMU_GYRO_FSR_DPS * DEG_TO_RAD / ADC_SCALE_16BIT;

        data->accel_x = (float)rx_data.accel_raw[0] * accel_scale;
        data->accel_y = (float)rx_data.accel_raw[1] * accel_scale;
        data->accel_z = (float)rx_data.accel_raw[2] * accel_scale;
        data->gyro_x = (float)rx_data.gyro_raw[0] * gyro_scale;
        data->gyro_y = (float)rx_data.gyro_raw[1] * gyro_scale;
        data->gyro_z = (float)rx_data.gyro_raw[2] * gyro_scale;

        data->timestamp = rx_data.timestamp;
        return 0;
    }
    return INV_ERROR_TIMEOUT;
}