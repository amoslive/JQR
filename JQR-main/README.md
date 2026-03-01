# Waist-Wheelbot (ROS2 + Simulation)
**A waist-actuated wheel robot supporting 2-wheel / 4-wheel driving and active jumping in 2-wheel mode.**

![jump demo](assets/jump_long.gif)

> Private research repo for academic review (real robot + simulation + recorded data).

## Repository layout
- `ros2_ws/` : ROS2 workspace (real robot + ROS tools)
- `simulators/matlab/` : MATLAB/Simulink dynamics & analysis
- `simulators/webots/` : Webots simulation project
- `simulators/mujoco/` : MuJoCo model + scripts
- `data/` : datasets (rosbag2 / logs) **not stored directly in git**
- `docs/` : additional documentation

## Quickstart
### A) ROS2 (real robot / core stack)
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
# example launch:
ros2 launch wheelbot_bringup bringup.launch.py
