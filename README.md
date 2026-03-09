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
. install/setup.bash
  . can_init.sh && ros2 run py01_topic canopen_motor_node
ros2 launch waistcar_control launch.xml
ros2 run unitree_control unitree_motor_node 
//Kill the Program
screen -ls |grep ached | cut -d. -f1| awk '{print $1}' | xargs kill

