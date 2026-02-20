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

Navigation bringup (display + Nav2)

```bash
ros2 launch batbot_bringup nav_bringup.launch.py
```

Navigation bringup without namespace

```bash
ros2 launch batbot_bringup nav_bringup.launch.py \
  robot_ns:=batbot \
  use_namespace:=False
```

Optional custom map:

```bash
ros2 launch batbot_bringup nav_bringup.launch.py \
  robot_ns:=batbot \
  map:=/path/to/map.yaml
```

RViz navigation workflow

1. Click `2D Pose Estimate` and drag to set initial pose.
2. Click `Nav2 Goal` and drag to send a goal.

Quick checks when robot does not move

```bash
ros2 lifecycle get /bt_navigator
ros2 action list | grep navigate
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /batbot/cmd_vel
```
