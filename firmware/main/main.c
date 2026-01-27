// ============================================================================
// Standard C Library Headers
// ============================================================================
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// ============================================================================
// ROS / micro-ROS Headers
// ============================================================================
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <uros_network_interfaces.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// ============================================================================
// ESP-IDF Headers
// ============================================================================
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// ============================================================================
// Project Specific Headers
// ============================================================================
#include "beep.h"
#include "imu.h"
#include "lidar_ms200.h"
#include "motor.h"
#include "motion.h"
#include "pwm_motor.h"
#include "state.h"

static const char* TAG = "MAIN";

#define RCCHECK(fn)                                                                             \
    do {                                                                                        \
        rcl_ret_t temp_rc = fn;                                                                 \
        if ((temp_rc != RCL_RET_OK)) {                                                          \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                                  \
        }                                                                                       \
    } while (0)
#define RCSOFTCHECK(fn)                                                                           \
    do {                                                                                          \
        rcl_ret_t temp_rc = fn;                                                                   \
        if ((temp_rc != RCL_RET_OK)) {                                                            \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                         \
    } while (0)

static uint64_t time_offset_us = 0; // us
static uint64_t last_sync_time = 0; // us
static uint64_t last_ping_time = 0; // us

static void sync_time(const int timeout_ms)
{
    if (rmw_uros_sync_session(timeout_ms) == RCL_RET_OK) {
        uint64_t now = esp_timer_get_time();
        uint64_t ros_time_us = rmw_uros_epoch_nanos() / 1000ULL;
        time_offset_us = ros_time_us - now;
        last_sync_time = now;
        ESP_LOGI(TAG, "Time synced. Offset: %llu us", time_offset_us);
    } else {
        ESP_LOGW(TAG, "Time sync failed");
    }
}

// TODO:
void subscription_callback_test(const void* msgin)
{
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    ESP_LOGI(TAG, "Received: %d\n", (int)msg->data);
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

void subscription_callback_twist(const void* msgin)
{
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    ESP_LOGI(TAG, "cmd_vel:%.2f, %.2f, %.2f", msg->linear.x, msg->linear.y, msg->angular.z);
    motion_cmd_t cmd = {
        .Vx = msg->linear.x,
        .Vy = msg->linear.y,
        .Wz = msg->angular.z,
    };
    if (motion_ctrl(&cmd) != ESP_OK) {
        ESP_LOGE(TAG, "Motion ctrl failed");
    }
}

static sensor_msgs__msg__Imu imu_msg;
static rcl_publisher_t imu_publisher;

// only init use
static void imu_data_init(void)
{
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_frame");

    imu_publisher = rcl_get_zero_initialized_publisher();
}

void imu_timer_publisher(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    static int err_count = 0;
    imu_data_t data;
    for (int i = 0; i < 5; i++) {
        int rc = imu_wait_for_data(&data, 0);
        if (rc) {
            break;
        }

        imu_msg.linear_acceleration.x = data.accel_x;
        imu_msg.linear_acceleration.y = data.accel_y;
        imu_msg.linear_acceleration.z = data.accel_z;
        imu_msg.angular_velocity.x = data.gyro_x;
        imu_msg.angular_velocity.y = data.gyro_y;
        imu_msg.angular_velocity.z = data.gyro_z;

        uint64_t now_us = data.timestamp + time_offset_us;
        imu_msg.header.stamp.sec = now_us / 1000000;
        imu_msg.header.stamp.nanosec = (now_us % 1000000) * 1000;

        rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
        if (rc != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to publish IMU message: %d", err_count++);
        } else {
            err_count = 0;
        }
    }
}

static sensor_msgs__msg__LaserScan lidar_msg;
static rcl_publisher_t lidar_publisher;

// only init use
static esp_err_t lidar_data_init(void)
{
    sensor_msgs__msg__LaserScan__init(&lidar_msg);
    rosidl_runtime_c__String__assign(&lidar_msg.header.frame_id, "laser_frame");
    lidar_msg.angle_min = -180 * M_PI / 180.0;
    lidar_msg.angle_max = 180 * M_PI / 180.0;

    lidar_msg.angle_increment = 1 * M_PI / 180.0;
    lidar_msg.range_min = 0.12;
    lidar_msg.range_max = 8.0;

    bool success = rosidl_runtime_c__float__Sequence__init(&lidar_msg.ranges, MS200_POINT_MAX);
    if (!success) {
        ESP_LOGE(TAG, "Failed to allocate memory for ranges");
        return ESP_ERR_NO_MEM;
    }
    success = rosidl_runtime_c__float__Sequence__init(&lidar_msg.intensities, MS200_POINT_MAX);
    if (!success) {
        ESP_LOGE(TAG, "Failed to allocate memory for intensities");
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < MS200_POINT_MAX; i++) {
        lidar_msg.intensities.data[i] = 0;
        lidar_msg.ranges.data[i] = 0;
    }

    lidar_publisher = rcl_get_zero_initialized_publisher();
    return ESP_OK;
}

void lidar_timer_publisher(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    static int err_count = 0;
    ms200_frame_t frame;
    for (int retry = 0; retry < 3; retry++) { // try 3 times
        int rc = lidar_ms200_read(&frame, 0);
        if (rc != ESP_OK) {
            break;
        }
        ESP_LOGD(TAG,
                 "%5llu us Got [%d mm][%d] end",
                 frame.timestamp,
                 frame.points[0].distance,
                 frame.points[0].intensity);
        for (int i = 0; i < MS200_POINT_MAX; i++) {
            int raw_index = (i + 180) % MS200_POINT_MAX;
            lidar_msg.ranges.data[i] = frame.points[raw_index].distance / 1000.0; // mm to m
            lidar_msg.intensities.data[i] = frame.points[raw_index].intensity;
        }

        uint64_t now_us = frame.timestamp + time_offset_us;
        lidar_msg.header.stamp.sec = now_us / 1000000;
        lidar_msg.header.stamp.nanosec = (now_us % 1000000) * 1000;

        rc = rcl_publish(&lidar_publisher, &lidar_msg, NULL);
        if (rc != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to publish Laser message: %d", err_count++);
        } else {
            err_count = 0;
        }
    }
}

static nav_msgs__msg__Odometry odom_msg;
static rcl_publisher_t odom_publisher;

static void odom_data_init(void)
{
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom_frame");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint");

    for (int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;  // x
    odom_msg.pose.covariance[7] = 0.01;  // y
    odom_msg.pose.covariance[35] = 0.01; // yaw (approx)

    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_publisher = rcl_get_zero_initialized_publisher();
}

// only use yaw
static void odom_euler_to_quat(float yaw, geometry_msgs__msg__Quaternion* q)
{
    const float cos_0 = cos(0);
    const float sin_0 = sin(0);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos_0; // cos(pitch * 0.5)
    float sp = sin_0; // sin(pitch * 0.5)
    float cr = cos_0; // cos(roll * 0.5)
    float sr = sin_0; // sin(roll * 0.5)

    q->x = cy * cp * cr + sy * sp * sr;
    q->y = cy * cp * sr - sy * sp * cr;
    q->z = sy * cp * sr + cy * sp * cr;
    q->w = sy * cp * cr - cy * sp * sr;
}

void odom_timer_publisher(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    static int err_count = 0;
    static uint64_t last_time = 0;
    uint64_t now = esp_timer_get_time();
    if (last_time == 0) {
        last_time = now; // first skip
        return;
    }
    float dt = (now - last_time) / 1000000.0f;
    last_time = now;
    if (dt < 0)
        dt = 0;

    motion_cmd_t speed;
    motion_get_speed(&speed);

    static float x = 0;
    static float y = 0;
    static float yaw = 0;

    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);
    float delta_x = (speed.Vx * cos_yaw - speed.Vy * sin_yaw) * dt; // m
    float delta_y = (speed.Vx * sin_yaw + speed.Vy * cos_yaw) * dt; // m

    x += delta_x;
    y += delta_y;
    yaw += speed.Wz * dt;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_euler_to_quat(yaw, &odom_msg.pose.pose.orientation);

    odom_msg.twist.twist.linear.x = speed.Vx;
    odom_msg.twist.twist.linear.y = speed.Vy;
    odom_msg.twist.twist.angular.z = speed.Wz;

    uint64_t sys_us = now + time_offset_us;
    odom_msg.header.stamp.sec = sys_us / 1000000;
    odom_msg.header.stamp.nanosec = (sys_us % 1000000) * 1000;
    rcl_ret_t rc = rcl_publish(&odom_publisher, &odom_msg, NULL);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to publish Odom messages: %d", err_count++);
    } else {
        err_count = 0;
    }
}

static esp_err_t data_init(void)
{
    imu_data_init();
    esp_err_t rc = lidar_data_init();
    if (rc != ESP_OK) {
        return rc;
    }
    odom_data_init();
    return ESP_OK;
}

#define RC_GOTO(fn, label, msg)                                                                      \
    do {                                                                                             \
        rcl_ret_t rc = (fn);                                                                         \
        if (rc != RCL_RET_OK) {                                                                      \
            ESP_LOGE(TAG, "[Line %d] %s failed (Ret: %d). GOTO -> " #label, __LINE__, msg, (int)rc); \
            goto label;                                                                              \
        }                                                                                            \
    } while (0)

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

    rcl_subscription_t subscriber_twist;
    geometry_msgs__msg__Twist msg_twist;

    rcl_timer_t timer_imu;
    rcl_timer_t timer_lidar;
    rcl_timer_t timer_odom;

    app_state(APP_STATE_DISCONN);
    while (1) {
        init_options = rcl_get_zero_initialized_init_options();

        ESP_LOGD(TAG, "Initializing micro-ROS ...");
        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init init_options");
            vTaskDelay(pdMS_TO_TICKS(1000));
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
        subscriber_twist = rcl_get_zero_initialized_subscription();
        executor = rclc_executor_get_zero_initialized_executor();
        timer_imu = rcl_get_zero_initialized_timer();
        timer_lidar = rcl_get_zero_initialized_timer();
        timer_odom = rcl_get_zero_initialized_timer();
        memset(&support, 0, sizeof(rclc_support_t));

        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to init support, retrying...");
            RCSOFTCHECK(rcl_init_options_fini(&init_options));
            continue;
        }

        RC_GOTO(rclc_node_init_default(&node, "actuator", CONFIG_MICRO_ROS_NAMESPACE, &support),
                cleanup,
                "Failed to init node");

        RC_GOTO(rclc_subscription_init_default(
                    &subscriber_test, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "test"),
                cleanup,
                "Failed to init subscriber");
        RC_GOTO(
            rclc_subscription_init_default(
                &subscriber_twist, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"),
            cleanup,
            "Failed to init subscriber");

        RC_GOTO(rclc_publisher_init_default(
                    &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"),
                cleanup,
                "Failed to init imu publisher");
        RC_GOTO(rclc_timer_init_default2(&timer_imu, &support, RCL_MS_TO_NS(10), imu_timer_publisher, true),
                cleanup, // 100Hz
                "Failed to init imu timer");

        RC_GOTO(
            rclc_publisher_init_default(
                &lidar_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"),
            cleanup,
            "Failed to init laser publisher");
        RC_GOTO(
            rclc_timer_init_default2(&timer_lidar, &support, RCL_MS_TO_NS(100), lidar_timer_publisher, true),
            cleanup, // 10Hz
            "Failed to init laser timer");

        RC_GOTO(rclc_publisher_init_default(
                    &odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom_raw"),
                cleanup,
                "Failed to init odom publisher");
        RC_GOTO(rclc_timer_init_default2(&timer_odom, &support, RCL_MS_TO_NS(50), odom_timer_publisher, true),
                cleanup, // 20Hz
                "Failed to init odom timer");

        // 2 Subscription + 3 Timer
        RC_GOTO(rclc_executor_init(&executor, &support.context, 5, &allocator),
                cleanup,
                "Failed to init executor");
        RC_GOTO(rclc_executor_add_subscription(
                    &executor, &subscriber_test, &msg_test, &subscription_callback_test, ON_NEW_DATA),
                cleanup,
                "Failed to add test subscriber");
        RC_GOTO(rclc_executor_add_subscription(
                    &executor, &subscriber_twist, &msg_twist, &subscription_callback_twist, ON_NEW_DATA),
                cleanup,
                "Failed to add twist subscriber");
        RC_GOTO(rclc_executor_add_timer(&executor, &timer_imu), cleanup, "Failed to add imu timer");
        RC_GOTO(rclc_executor_add_timer(&executor, &timer_lidar), cleanup, "Failed to add lidar timer");
        RC_GOTO(rclc_executor_add_timer(&executor, &timer_odom), cleanup, "Failed to add odom timer");

        sync_time(1000);

        ESP_LOGI(TAG, "Micro-ROS task running!");
        app_state(APP_STATE_OK);
        while (1) {
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
                ESP_LOGI(TAG, "Connection lost/Error in spin. Resetting...");
                break;
            }
            uint64_t now = esp_timer_get_time();
            if ((now - last_sync_time) > 300000000LL) { // 5 minutes
                sync_time(100);
                last_sync_time = now;
            }
            if ((now - last_ping_time) > 5000000) { // 5 seconds
                if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
                    ESP_LOGW(TAG, "Agent ping failed, resetting...");
                    break;
                }
                last_ping_time = now;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    cleanup:
        app_state(APP_STATE_DISCONN);
        ESP_LOGI(TAG, "Cleaning up resources...\n");
        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_subscription_fini(&subscriber_test, &node));
        RCSOFTCHECK(rcl_subscription_fini(&subscriber_twist, &node));
        RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
        RCSOFTCHECK(rcl_publisher_fini(&lidar_publisher, &node));
        RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
        RCSOFTCHECK(rcl_timer_fini(&timer_imu));
        RCSOFTCHECK(rcl_timer_fini(&timer_lidar));
        RCSOFTCHECK(rcl_timer_fini(&timer_odom));
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
    ESP_ERROR_CHECK(data_init());

    app_state_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(imu_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(lidar_ms200_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(motion_init());

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(
        micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

    // Init over
    beep_on_time(200);
}
