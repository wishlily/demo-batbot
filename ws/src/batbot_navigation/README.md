# BatBot Navigation

Build

```bash
cd ws
colcon build --packages-select batbot_navigation
```

Run localization (map_server + AMCL)

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_navigation localization.launch.py
```

Run full navigation stack (Nav2 + localization)

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_navigation navigation.launch.py
```

Run full navigation stack with process-based mode (debug-friendly)

```bash
ros2 launch batbot_navigation navigation.launch.py use_composition:=false
```

Use a custom map yaml

```bash
ros2 launch batbot_navigation localization.launch.py \
  map:=/absolute/path/to/map.yaml
```

Use a custom map for full navigation

```bash
ros2 launch batbot_navigation navigation.launch.py \
  map:=/absolute/path/to/map.yaml
```

AMCL parameters

- Default file: `config/amcl.yaml`
- Override at launch:

```bash
ros2 launch batbot_navigation localization.launch.py \
  amcl_params_file:=/absolute/path/to/amcl.yaml
```

Notes

- Expected frames: `map -> odom -> base_link` and `base_link -> laser`
- Localization/Navigate scan topic: `/batbot/scan`
- Nav2 default file: `config/nav2.yaml`
- Nav2 defaults: `/batbot/scan`, `/batbot/odom`, output `cmd_vel` to `/batbot/cmd_vel`

Navigation sanity checks

```bash
ros2 lifecycle get /bt_navigator
ros2 action list | grep navigate
ros2 topic hz /plan
ros2 topic hz /cmd_vel_nav
ros2 topic hz /batbot/cmd_vel
```
