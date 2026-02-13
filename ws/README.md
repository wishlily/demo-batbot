# BatBot ROS2 Project

BatBot is a ROS2-based robot project with robot description, simulation, control, and communication features.

## System Requirements

- ROS 2 (Jazzy or later)
- Colcon build tool
- Gazebo (for simulation)
- RViz (for visualization)

## Project Structure

```
ws/
├── src/
│   ├── batbot_bringup/      # BatBot bringup package
│   ├── batbot_description/  # BatBot robot description
│   ├── batbot_msgs/         # BatBot message definitions
│   ├── batbot_sim/          # BatBot simulation environment
│   └── batbot_teleop/       # BatBot teleop nodes
├── install/                 # Installation directory
├── log/                     # Build logs
└── script/                  # Script tools
```

## Installation

1. Clone or download the project to your workspace

2. Build all packages:
```bash
cd ws
colcon build --symlink-install
source install/setup.zsh
```

3. Verify installation:
```bash
ros2 pkg list | grep batbot
```

## Package Descriptions

### batbot_bringup

BatBot bringup package providing robot startup and configuration functionality.

**Build:**
```bash
colcon build --packages-select batbot_bringup
```

**Display robot motion in RViz:**
```bash
source install/setup.zsh
ros2 launch batbot_bringup display.launch.py
```

### batbot_description

BatBot robot description package containing URDF models and RViz configurations.

**Build:**
```bash
colcon build --packages-select batbot_description
```

**Display in RViz for testing:**
```bash
source install/setup.zsh
ros2 launch batbot_description display.launch.py
```

### batbot_msgs

BatBot message definitions package defining message types for robot communication.

**View message types:**
```bash
ros2 interface show batbot_msgs/msg/Config
```

### batbot_sim

BatBot simulation package providing Gazebo simulation environment and scene models.

**Build:**
```bash
colcon build --packages-select batbot_sim
```

**Run simulation in Gazebo:**
```bash
source install/setup.zsh
ros2 launch batbot_sim gz_sim.launch.py
```

### batbot_teleop

BatBot teleop package supporting both keyboard and joystick control.

**Build:**
```bash
colcon build --packages-select batbot_teleop
```

**Keyboard control:**
```bash
source install/setup.zsh
ros2 run batbot_teleop keyboard_teleop --ros-args -r __ns:=<namespace>
```

**Joystick control:**
```bash
source install/setup.zsh
ros2 launch batbot_teleop joy_teleop.launch.py
```


## Common Commands

### Full build
```bash
colcon build --symlink-install
source install/setup.zsh
```

### Build single package
```bash
colcon build --packages-select <package_name>
```

### Clean build
```bash
colcon clean
rm -rf install/ log/
```

## License

Apache-2.0

