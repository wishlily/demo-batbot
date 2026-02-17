# BatBot BringUp

Build

```bash
cd ws
colcon build --packages-select batbot_bringup
```

Display bringup (RViz + EKF, no SLAM)

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_bringup display.launch.py robot_ns:=batbot
```

Mapping bringup (display + slam_toolbox)

```bash
ros2 launch batbot_bringup mapping.launch.py robot_ns:=batbot
```

`mapping.launch.py` uses `config/rviz/mapping.rviz` by default:
- Fixed Frame: `map`
- Map display topic: `/map`
