#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "state.h"

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

std_msgs__msg__Int32 recv_msg;

void subscription_callback(const void* msgin)
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
}

void micro_ros_task(void* arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_subscription_t subscriber;
    rclc_executor_t executor;
    rcl_init_options_t init_options;

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
        rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
#endif

        ESP_LOGI(TAG, "Connecting to agent...");
        while (rmw_uros_ping_agent_options(1000, 1, rmw_options) != RCL_RET_OK) {
            ESP_LOGI(TAG,
                     "Waiting for agent connection (%s:%s)...",
                     CONFIG_MICRO_ROS_AGENT_IP,
                     CONFIG_MICRO_ROS_AGENT_PORT);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG, "Agent connected!");

        node = rcl_get_zero_initialized_node();
        subscriber = rcl_get_zero_initialized_subscription();
        executor = rclc_executor_get_zero_initialized_executor();
        memset(&support, 0, sizeof(rclc_support_t));

        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init support, retrying...");
            RCSOFTCHECK(rcl_init_options_fini(&init_options));
            continue;
        }

        if (rclc_node_init_default(&node, "main", CONFIG_MICRO_ROS_NAMESPACE, &support) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init node");
            goto cleanup;
        }
        if (rclc_subscription_init_default(
                &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "subscriber") !=
            RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init subscriber");
            goto cleanup;
        }

        if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init executor");
            goto cleanup;
        }
        if (rclc_executor_add_subscription(
                &executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to add subscription");
            goto cleanup;
        }

        ESP_LOGI(TAG, "Micro-ROS task running!");
        while (1) {
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) {
                ESP_LOGI(TAG, "Connection lost/Error in spin. Resetting...");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    cleanup:
        ESP_LOGI(TAG, "Cleaning up resources...\n");
        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
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

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(
        micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
