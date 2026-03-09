

# Waist-Actuated Wheel Robot (Waist-Wheelbot)

A waist-actuated wheel robot capable of **2-wheel balancing**, **4-wheel
driving**, and **active jumping in 2-wheel mode**.

This repository contains the **control stack, simulation environments,
dynamic modeling, and experimental logs** developed for my undergraduate
thesis.


> Private research repository for academic review.

------------------------------------------------------------------------

## Overview

The robot is a **waist-actuated wheel platform** designed to explore
dynamic locomotion and balancing control.

The system supports multiple operating modes:

-   Two-wheel balancing
-   Four-wheel driving
-   Active jumping in two-wheel mode
-   Waist-actuated attitude control

The project integrates:

-   ROS2 real robot control
-   Dynamic modeling and analysis
-   Multi-platform simulation
-   Experimental validation

------------------------------------------------------------------------

## Hardware Platform

The robot consists of:

-   Rear wheel drive motor
-   Waist actuation joint
-   Two-link body structure
-   IMU for attitude estimation
-   CANOpen motor drivers
-   Unitree motor interface

The robot can switch between **2-wheel and 4-wheel modes** depending on
task requirements.

------------------------------------------------------------------------

## Repository layout
- `ros2_ws/` : ROS2 workspace (real robot + ROS tools)
- `simulators/matlab/` : MATLAB/Simulink dynamics & analysis
- `simulators/webots/` : Webots simulation project
- `simulators/mujoco/` : MuJoCo model + scripts
- `data/` : datasets 
- `docs/` : additional documentation

------------------------------------------------------------------------

## Quickstart

### ROS2 (Real Robot Control)

Build the ROS2 workspace:

``` bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Initialize CAN interface and start motor driver:

``` bash
./can_init.sh
ros2 run py01_topic canopen_motor_node
```

Launch the control system:

``` bash
ros2 launch waistcar_control launch.xml
```

Start Unitree motor interface:

``` bash
ros2 run unitree_control unitree_motor_node
```



------------------------------------------------------------------------

## Experimental Results

Example robot jumping experiment:

![jump demo](assets/jump_long.gif)

Additional experiments include:

-   balancing control
-   turning motion
-   motor torque tracking
-   jumping experiments

Experimental logs are stored in `data/`.

------------------------------------------------------------------------



## License

This repository is intended for **academic research and thesis review**.

------------------------------------------------------------------------

## Author

Master Thesis Project

Research on Jumping Mechanism and
Control Methods of Reconfigurable
Robots\
2026

