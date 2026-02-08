#!/bin/zsh
echo "[env] project env loaded"

ros2_env() {
    export WEBOTS_HOME=/usr/local/webots
    export GZ_CONFIG_PATH=/usr/share/gz
    export ROS_DOMAIN_ID=20
    source /opt/ros/jazzy/setup.zsh
}

ros2_env