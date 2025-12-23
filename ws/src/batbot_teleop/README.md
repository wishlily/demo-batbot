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
ros2 run batbot_teleop keyboard_teleop
```

Joystick Teleoperation for the BatBot

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_teleop joy_teleop.launch.py
```