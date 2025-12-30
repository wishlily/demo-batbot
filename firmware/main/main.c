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

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

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

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        printf("Publishing: %d\n", (int)msg.data);
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

void micro_ros_task(void* arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // change ros domain id
    RCCHECK(rcl_init_options_set_domain_id(&init_options, CONFIG_MICRO_ROS_DOMAIN_ID));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", CONFIG_MICRO_ROS_NAMESPACE, &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "freertos_int32_publisher"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback, true));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void print_system_info(void)
{
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }
    printf("%" PRIu32 "MB %s flash\n",
           flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}

void app_main(void)
{
    print_system_info();
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(
        micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
