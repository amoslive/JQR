# Hardware Platform

This project is based on a **waist-actuated wheel robot** designed for dynamic balancing and jumping experiments.

The robot features a **two-link body structure with a waist joint**, enabling attitude regulation and dynamic motion control.

---

# Mechanical Structure

The robot consists of the following main components:

- Rear wheel drive module
- Waist actuation joint
- Two-link body structure
- Four-wheel support configuration
- Structural frame and mounting brackets

The robot can operate in two modes:

**1. Two-wheel mode**

- Used for dynamic balancing and jumping
- Rear wheel provides locomotion
- Waist joint regulates body attitude

**2. Four-wheel mode**

- Used for stable driving
- Provides additional ground support

---

# Actuators

The robot uses the following actuators:

### Rear Wheel Motor

Responsible for:

- locomotion
- balancing control

The motor is controlled through a **CANOpen motor driver**.

### Waist Joint Motor

Responsible for:

- upper body attitude control
- dynamic motion generation (jumping)

The waist actuator enables active body motion.

---

# Sensors

The robot uses the following sensors:

### IMU

An **Inertial Measurement Unit (IMU)** is used to measure:

- body pitch angle
- angular velocity

The IMU provides feedback for the balancing controller.

---

# Control Hardware

The robot control system includes:

- onboard computer running **ROS2**
- CAN interface for motor communication
- power management module

The control framework is implemented using **ROS2 nodes**.

---

# Experimental Platform

The hardware platform is used for:

- balancing control experiments
- turning motion experiments
- motor torque tracking
- dynamic jumping experiments

Experimental logs are recorded using **rosbag2**.
