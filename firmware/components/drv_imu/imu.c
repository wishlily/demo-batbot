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
#include "imu_fusion.h"

#define LOG_THROTTLE(tag, level, interval_ms, format, ...)                                 \
    do {                                                                                   \
        static int64_t last_log_time = 0;                                                  \
        static uint32_t occur_count = 0;                                                   \
        occur_count++;                                                                     \
        int64_t now = esp_timer_get_time() / 1000;                                         \
        if (now - last_log_time > interval_ms) {                                           \
            ESP_LOG_LEVEL(level, tag, format " [Count: %lu]", ##__VA_ARGS__, occur_count); \
            last_log_time = now;                                                           \
            occur_count = 0;                                                               \
        }                                                                                  \
    } while (0)

#define LOGW_THROTTLE(tag, interval_ms, format, ...) \
    LOG_THROTTLE(tag, ESP_LOG_WARN, interval_ms, format, ##__VA_ARGS__)

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
static imu_fusion_handle_t imu_fusion_dev;
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
    if (IMU_RATE == 50) {
        IMU_ERROR_CHECK(inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ODR_50_HZ));
        IMU_ERROR_CHECK(inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_ODR_50_HZ));
    } else {
        return ESP_ERR_INVALID_ARG;
    }

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
    static uint16_t downsample_rate = 0;
    static uint64_t last_timestamp = 0;
    uint64_t int_timestamp = 0;
    imu_data_t data;

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
        data.timestamp = int_timestamp;
    } else {
        data.timestamp = event->timestamp_fsync + rollover_num * UINT16_MAX;
        data.timestamp *= inv_imu_get_timestamp_resolution_us(&imu_dev);
    }

    uint64_t current_time = data.timestamp;
    if (last_timestamp == 0) {
        last_timestamp = current_time;
        return;
    }
    float dt = (current_time - last_timestamp) / 1e6f; // seconds
    last_timestamp = current_time;

    imu_fusion_data_t out;
    if (imu_fusion_process_quaternion(&imu_fusion_dev, event->accel, event->gyro, &out, dt)) {
        return;
    }

    atomic_store(&imu_temperature_raw, event->temperature);

    if ((downsample_rate++) % IMU_DOWNSAMPLE_RATE != 0) {
        return;
    }

    data.accel_x = out.accel_mss[0];
    data.accel_y = out.accel_mss[1];
    data.accel_z = out.accel_mss[2];
    data.gyro_x = out.gyro_rads[0];
    data.gyro_y = out.gyro_rads[1];
    data.gyro_z = out.gyro_rads[2];
    data.q[0] = out.q[0];
    data.q[1] = out.q[1];
    data.q[2] = out.q[2];
    data.q[3] = out.q[3];

    BaseType_t ret = xQueueSend(imu_raw_queue, &data, 0);
    if (ret != pdTRUE) {
        LOGW_THROTTLE(IMU_LOG_TAG, 1000, "Queue full, packet dropped.");
    }
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
    imu_raw_queue = xQueueCreate(IMU_FIFO_SIZE, sizeof(imu_data_t));
    if (imu_raw_queue == NULL) {
        return INV_ERROR_MEM;
    }
    imu_fusion_init(&imu_fusion_dev, IMU_RATE, 0.5, IMU_ACCEL_FSR_G, IMU_GYRO_FSR_DPS);

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
#define DEG_TO_RAD (M_PI / 180.0f)

// return 0 on success
// return INV_ERROR_TIMEOUT if timeout
int imu_wait_for_data(imu_data_t* data, int32_t ticks_to_wait)
{
    if (xQueueReceive(imu_raw_queue, data, ticks_to_wait) == pdTRUE) {
        return 0;
    }
    return ESP_ERR_TIMEOUT;
}