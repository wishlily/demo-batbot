#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>

#include <rosidl_runtime_c/string_functions.h>
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"

#include "state.h"
#include "beep.h"
#include "imu.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char* TAG = "MAIN";

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK)) {                                                   \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK)) {                                                     \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

static uint64_t time_offset_us = 0; // us

static void sync_time(void)
{
    const int timeout_ms = 1000;
    if (rmw_uros_sync_session(timeout_ms) == RCL_RET_OK) {
        uint64_t now = esp_timer_get_time();
        uint64_t ros_time_us = rmw_uros_epoch_nanos() / 1000ULL;
        time_offset_us = ros_time_us - now;
        ESP_LOGI(TAG, "Time synced. Offset: %llu us", time_offset_us);
    } else {
        ESP_LOGW(TAG, "Time sync failed");
    }
}

// TODO:
void subscription_callback_test(const void* msgin)
{
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    printf("Received: %d\n", (int)msg->data);
    switch (msg->data) {
    case APP_STATE_TEST:
        app_state(APP_STATE_TEST);
        break;
    default:
        app_state(msg->data);
        break;
    }
    beep_on_time(msg->data);
}

static void msg_imu_init(sensor_msgs__msg__Imu* msg)
{
    sensor_msgs__msg__Imu__init(msg);
    rosidl_runtime_c__String__assign(&msg->header.frame_id, "imu_link");
}

int check_publisher_imu(rcl_publisher_t* pub, sensor_msgs__msg__Imu* msg)
{
    imu_data_t data;
    int count = 0;
    while (count < 5 && imu_wait_for_data(&data, 0) == 0) {
        // static int count = 0;
        // if (count++ % 50 == 0) {
        //     ESP_LOGI(
        //         TAG,
        //         "%5llu us, Accel: % 8.2f % 8.2f % 8.2f m/s^2, Gyro: % 8.2f % 8.2f % 8.2f, Temp: % 4.2f
        //         degC", data->timestamp, data->accel_x, data->accel_y, data->accel_z, data->gyro_x,
        //         data->gyro_y,
        //         data->gyro_z,
        //         imu_get_temperature());
        // }
        count++;

        msg->linear_acceleration.x = data.accel_x;
        msg->linear_acceleration.y = data.accel_y;
        msg->linear_acceleration.z = data.accel_z;
        msg->angular_velocity.x = data.gyro_x;
        msg->angular_velocity.y = data.gyro_y;
        msg->angular_velocity.z = data.gyro_z;

        uint64_t now_us = data.timestamp + time_offset_us;
        msg->header.stamp.sec = now_us / 1000000;
        msg->header.stamp.nanosec = (now_us % 1000000) * 1000;

        if (rcl_publish(pub, msg, NULL)) {
            return RCL_RET_ERROR;
        }
    }
    return RCL_RET_OK;
}

static uint32_t micro_ros_client_key(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t client_key = (uint32_t)(mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5]);
    return client_key;
}

void micro_ros_task(void* arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_init_options_t init_options;

    rcl_subscription_t subscriber_test;
    std_msgs__msg__Int32 msg_test;

    rcl_publisher_t publisher_imu;
    sensor_msgs__msg__Imu msg_imu;

    while (1) {
        init_options = rcl_get_zero_initialized_init_options();

        ESP_LOGD(TAG, "Initializing micro-ROS ...");
        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init init_options");
            continue;
        }
        if (rcl_init_options_set_domain_id(&init_options, CONFIG_MICRO_ROS_DOMAIN_ID) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to set Domain ID");
            RCSOFTCHECK(rcl_init_options_fini(&init_options));
            continue;
        }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCSOFTCHECK(rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

        RCSOFTCHECK(rmw_uros_options_set_client_key(micro_ros_client_key(), rmw_options));

        ESP_LOGI(TAG, "Connecting to agent...");
        while (rmw_uros_ping_agent_options(1000, 1, rmw_options) != RCL_RET_OK) {
            ESP_LOGI(TAG,
                     "Waiting for agent connection (%s:%s)...",
                     CONFIG_MICRO_ROS_AGENT_IP,
                     CONFIG_MICRO_ROS_AGENT_PORT);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG, "Agent connected!");
#endif

        node = rcl_get_zero_initialized_node();
        subscriber_test = rcl_get_zero_initialized_subscription();
        publisher_imu = rcl_get_zero_initialized_publisher();
        executor = rclc_executor_get_zero_initialized_executor();
        memset(&support, 0, sizeof(rclc_support_t));

        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init support, retrying...");
            RCSOFTCHECK(rcl_init_options_fini(&init_options));
            continue;
        }

        if (rclc_node_init_default(&node, "actuator", CONFIG_MICRO_ROS_NAMESPACE, &support) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init node");
            goto cleanup;
        }

        if (rclc_subscription_init_default(
                &subscriber_test, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "test") !=
            RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init subscriber");
            goto cleanup;
        }

        msg_imu_init(&msg_imu);
        if (rclc_publisher_init_default(
                &publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") !=
            RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init publisher");
            goto cleanup;
        }

        if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init executor");
            goto cleanup;
        }
        if (rclc_executor_add_subscription(
                &executor, &subscriber_test, &msg_test, &subscription_callback_test, ON_NEW_DATA) !=
            RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to add subscription");
            goto cleanup;
        }

        sync_time();

        ESP_LOGI(TAG, "Micro-ROS task running!");
        while (1) {
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK) {
                ESP_LOGI(TAG, "Connection lost/Error in spin. Resetting...");
                break;
            }
            RCSOFTCHECK(check_publisher_imu(&publisher_imu, &msg_imu));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    cleanup:
        ESP_LOGI(TAG, "Cleaning up resources...\n");
        sensor_msgs__msg__Imu__fini(&msg_imu);
        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_subscription_fini(&subscriber_test, &node));
        RCSOFTCHECK(rcl_publisher_fini(&publisher_imu, &node));
        RCSOFTCHECK(rcl_node_fini(&node));
        RCSOFTCHECK(rclc_support_fini(&support));
        RCSOFTCHECK(rcl_init_options_fini(&init_options));

        ESP_LOGI(TAG, "Reconnecting in 1s...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    app_state_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(imu_init());

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(
        micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
