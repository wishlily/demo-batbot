# Batbot ROS2 Firmware

A micro-ROS based firmware for ESP32 robot platform, implementing a complete ROS2 node for actuator control and sensor data publishing.

## Overview

This firmware provides a fully functional ROS2 node running on ESP32 that communicates with a micro-ROS agent via UDP. It publishes sensor data (IMU, Lidar, Odometry) at configurable rates and accepts velocity commands to control the robot's motion.

## Hardware Platform

- **Microcontroller**: ESP32 (Dual-core Xtensa LX7)
- **Framework**: ESP-IDF v5.x
- **Communication**: micro-ROS over UDP
- **ROS 2 Distribution**: Jazzy (DDS middleware)

## Features

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/imu_raw` | `sensor_msgs/msg/Imu` | 50 Hz | IMU sensor data (acceleration, angular velocity, orientation) |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | 360° laser scan data from MS200 lidar |
| `/odom_raw` | `nav_msgs/msg/Odometry` | 20 Hz | Odometry estimation from wheel encoders |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity commands |
| `/config` | `batbot_msgs/msg/Config` | Runtime configuration parameters |

## Components

### Core Components

- **app_motion**: Motion control logic for differential drive robot
- **app_state**: Application state management (connection, error, test states)
- **drv_imu**: ICM-42670-P IMU sensor driver with sensor fusion
- **drv_lidar**: MS200 LiDAR driver (360° range, 8m max distance)
- **drv_encoder**: Wheel encoder driver for odometry
- **drv_pid_motor**: PID-based motor speed control
- **drv_pwm_motor**: PWM motor driver
- **drv_uart**: UART communication driver
- **drv_led**: LED control driver
- **drv_beep**: Buzzer control driver
- **lib_morse**: Morse code library for status indication

### External Libraries

- **TDK ICM-42670-P Driver**: Inertial measurement unit driver
- **Fusion Library**: Sensor fusion algorithms for orientation estimation
- **micro-ROS ESP-IDF Component**: micro-ROS integration for ESP32

## Hardware Configuration

### Motor System

- **Drive Type**: Differential drive
- **Wheel Width**: 135 mm
- **Wheel Length**: 95 mm

## Building and Flashing

### Prerequisites

- ESP-IDF v5.x installed
- Python 3.8+
- ESP-IDF environment configured

### Build Steps

```bash
# Set the ESP-IDF path (if not already set)
export IDF_PATH=/path/to/esp-idf

# Clone the micro-ROS agent (for development)
git clone https://github.com/micro-ROS/micro_ros_agent.git
cd micro_ros_agent
pip install -r requirements.txt
python3 -m micro_ros_agent udp --port 8888

# In another terminal, build the firmware
cd firmware
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

### Configuration

The firmware supports configuration through Kconfig options:

```bash
idf.py menuconfig
```

Key configuration options:

- **micro-ROS example-app settings**
  - `MICRO_ROS_APP_STACK`: Stack size for micro-ROS task (default: 16000 bytes)
  - `MICRO_ROS_APP_TASK_PRIO`: Priority of micro-ROS task (default: 5)
  - `MICRO_ROS_DOMAIN_ID`: ROS domain ID (default: 20)
  - `MICRO_ROS_NAMESPACE`: ROS node namespace (default: "batbot")

## ROS2 Communication

### Connection Setup

1. Start the micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp --port 8888
   ```

2. The firmware will automatically connect to the agent using the configured IP and port.

3. Connection status is indicated by Morse code on LED:
   - **"OK"**: Connected
   - **"DISCONN"**: Disconnected
   - **"TEST"**: Test mode

### Time Synchronization

The firmware maintains time synchronization with the ROS2 agent using `rmw_uros_sync_session()`. Time offset is calculated and applied to all published messages.

### Reconnection Logic

The firmware implements automatic reconnection:
- Attempts to reconnect every 1 second if connection is lost
- Pings the agent every 5 seconds to maintain connectivity
- Resets micro-ROS resources and reinitializes on connection failure

## ROS2 Nodes

### Actuator Node

- **Node Name**: `actuator`
- **Namespace**: Configurable (default: `batbot`)
- **Domain ID**: Configurable (default: 20)

## Development

### Adding New Components

New components should follow the existing structure:
1. Create component directory in `components/`
2. Add `CMakeLists.txt` and `idf_component.yml`
3. Include component in `main/CMakeLists.txt`

### Component Structure

```
components/
├── app_motion/          # Motion control
├── app_state/           # State management
├── drv_beep/            # Buzzer driver
├── drv_encoder/         # Encoder driver
├── drv_imu/             # IMU driver
├── drv_led/             # LED driver
├── drv_lidar/           # LiDAR driver
├── drv_pid_motor/       # PID motor control
├── drv_pwm_motor/       # PWM motor driver
├── drv_uart/            # UART driver
├── lib_morse/           # Morse code library
└── micro_ros_espidf_component/  # micro-ROS component
```

## Troubleshooting

### Connection Issues

- Ensure the micro-ROS agent is running on the same network
- Check that the configured IP and port match the agent settings
- Verify the domain ID matches between firmware and agent

### Sensor Data Issues

- Check IMU initialization with `ESP_ERROR_CHECK_WITHOUT_ABORT(imu_init())`
- Verify LiDAR connection and power supply
- Check encoder wiring and configuration

### Build Issues

- Ensure ESP-IDF path is correctly set
- Run `idf.py fullclean` and rebuild if encountering issues
- Check that all dependencies are installed

## License

This project is part of the Batbot ROS2 firmware repository. Please refer to the LICENSE file for details.

## Acknowledgments

- micro-ROS team for the excellent ROS2 embedded framework
- TDK for the ICM-42670-P sensor driver
- Xio Technologies for the Fusion library
