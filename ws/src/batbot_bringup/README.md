# BatBot BringUp

Build

```bash
cd ws
colcon build --packages-select batbot_bringup
```

Display robot motion in RViz

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_bringup display.launch.py
```
