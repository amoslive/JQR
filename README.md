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

/////////////////////////////////////////////////////
# Waist-Actuated Wheel Robot (Waist-Wheelbot)

A waist-actuated wheel robot capable of **2-wheel balancing**, **4-wheel
driving**, and **active jumping in 2-wheel mode**.

This repository contains the **control stack, simulation environments,
dynamic modeling, and experimental logs** developed for my undergraduate
thesis.

![jump demo](assets/jump_long.gif)

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

## System Architecture

The overall system contains three main components.

### Real Robot Control (ROS2)

-   CANOpen motor drivers
-   Unitree motor interface
-   Balance controller
-   Waist actuation controller
-   ROS2 communication framework

### Simulation Environments

Multiple simulation platforms are used for development and validation.

-   **MATLAB / Simulink** -- dynamics modeling and controller analysis
-   **Webots** -- robot simulation environment
-   **MuJoCo** -- physics simulation for dynamic motion

### Experimental Data

Experimental logs include:

-   rosbag2 recordings
-   motor experiments
-   system identification data

------------------------------------------------------------------------

## Repository Layout

    ros2_ws/                 ROS2 workspace (real robot control stack)

    simulators/
      matlab/                MATLAB / Simulink dynamics modeling
      webots/                Webots simulation project
      mujoco/                MuJoCo robot model and scripts

    data/                    experimental datasets (rosbag2 / logs)
    docs/                    additional documentation
    assets/                  images and videos used in README

Large datasets are **not stored directly in the repository**.

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

## Simulation

### MATLAB / Simulink

Dynamic modeling and analysis are located in:

    simulators/matlab

Includes:

-   symbolic dynamic model
-   controller analysis
-   parameter computation

### Webots Simulation

    cd simulators/webots
    webots

### MuJoCo Simulation

    cd simulators/mujoco
    python run_sim.py

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

## Data

The `data/` folder contains references to experimental datasets.

Due to size limitations, **rosbag2 logs are not stored directly in this
repository**.

------------------------------------------------------------------------

## License

This repository is intended for **academic research and thesis review**.

------------------------------------------------------------------------

## Author

Bachelor Thesis Project

Waist-Actuated Wheel Robot\
2025
