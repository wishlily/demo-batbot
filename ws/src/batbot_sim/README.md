# BatBot Simulation

Build

```bash
cd ws
colcon build --packages-select batbot_sim
```

Use Gazebo for simulation

```bash
cd ws
source install/setup.zsh
ros2 launch batbot_sim gz_sim.launch.py
```