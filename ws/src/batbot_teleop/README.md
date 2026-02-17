# BatBot Teleop

Build

```bash
cd ws
colcon build --packages-select batbot_teleop
```

Keyboard Teleoperation for the BatBot

```bash
cd ws
source install/setup.zsh
ros2 run batbot_teleop keyboard_teleop --ros-args -r __ns:=<namespace>
```

Joystick Teleoperation for the BatBot

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_teleop joy_teleop.launch.py
```

Recommended smoother control for mapping:

```bash
ros2 launch batbot_teleop joy_teleop.launch.py \
  linear_speed_limit:=0.6 angular_speed_limit:=2.0 \
  linear_accel_limit:=0.5 angular_accel_limit:=1.2 \
  axis_deadzone:=0.15 cmd_publish_rate:=30.0
```
