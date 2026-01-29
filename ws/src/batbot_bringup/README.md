# BatBot BringUp

Build

```bash
cd ws
colcon build --packages-select batbot_bringup
```

Use RViz to display the car motion

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_bringup display.launch.py
```