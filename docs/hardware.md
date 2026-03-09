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



### Waist Joint Motor

Responsible for:

- upper body attitude control
- dynamic motion generation (jumping)

The waist actuator enables active body motion.
The motor is controlled through a **CANOpen motor driver**.

---
## Sensors

The robot uses the following sensors:

### IMU

An **Inertial Measurement Unit (IMU)** is used to measure:

- body pitch angle
- angular velocity

The IMU provides feedback for the balancing controller.

### Motor Encoders

Each motor provides **encoder feedback**, which is used to measure:

- wheel position
- wheel angular velocity
- waist joint position

The encoder signals are used for motion control and state estimation.

### Motor Torque Feedback

The motor drivers provide **torque feedback**, which is used to measure:

- wheel motor torque
- waist joint torque

Torque feedback is used for monitoring actuator performance and conducting motor torque experiments.

---

## Power System

The robot uses a multi-voltage power system to supply different components.

### Waist Motor Power

The waist motor is powered by a **72 V power supply**, provided by **three DJI FPV batteries connected in series**.

This high-voltage supply is required to support the torque demand of the waist actuator during dynamic motions such as jumping.

### Wheel Motor Power

The wheel motor is powered by a **24 V power supply**, provided by **a single DJI FPV battery**.

This supply is used for wheel locomotion and balancing control.

### Onboard Electronics

Low-voltage electronics, including:

- onboard computer
- control boards
- communication interfaces
- sensors

are powered by **5 V**, which is generated from the **24 V battery using a DC–DC converter**.

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

Experimental logs are recorded using **txt** and  **csv**.
