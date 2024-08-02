# OpenConvoy: Universal Platform for Real-World Testing of Cooperative Driving Systems
> Pre-print available on [Arxiv](https://arxiv.org/abs/2405.18600)

[Owen Burns](https://owenburns.co), [Hossein Maghsoumi](https://scholar.google.com/citations?user=z-xSxX0AAAAJ&hl=en), Israel Charles (https://www.linkedin.com/in/israel-charles/), [Yasser Fallah](https://www.ece.ucf.edu/person/yaser-p-fallah/)

> **Note**: WIP repo

## Conceptual Overview
OpenConvoy enables the testing of cooperative driving algorithms on miniature physical vehicles without the need to write motor control or communication code. 

### Cooperative Driving Algorithms
Cooperative driving algorithms typically have three components:
1. **Communication topology**: For an individual vehicle, this determines whether it should pay attention to--and therefore consider in its own movement calculation--a message it received from another vehicle. In an r-lookahead topology with r set to 2, if vehicle 4 received a message from vehicle 2 it would keep it, while it would disregard a message from vehicle 1. With OpenConvoy, any instance of *ROS1Control* or *ROS2Control* or a subclass of either has access to the position of the vehicle in the convoy, and by overriding *should_listen* one can implement an arbitrary communication topology.
2. **Spacing goal**: For an individual vehicle, this determines how it should use the messages it has received to determine its goal motion. The most common mimization objectives are cooperative adaptive cruise control, where the goal is to maintain a specific lead *time* to each vehicle, and platooning, where the goal is to maintain a specific lead *distance* to each vehicle. With OpenConvoy, you can override *minimization_objective* in any control subclass and implement a cost function to determine how well the hypothetical motion resulting from a candidate velocity and heading would align with your spacing goal.
3. **Controllers**: The controllers determine how the goal velocity and motion calculated by OpenConvoy based on the **Spacing goal** is turned into an instantaneous change of motion in the vehicle. By overriding *velocity_controller* and/or *heading_controller*, you can implement any control algorithm you want. By default, a Stanley controller is used for the heading controller and PID is used to control velocity.

### Hardware
To ensure that OpenConvoy works across the widest variety of hardware possible, we send movement commands as MavLink messages. **This necessitates that the vehicles used with OpenConvoy are equipped with an autopilot capable of receiving and handling MavLink messages.**

**Verified Hardware**:
- _ESCs_:
  - VXL-8s
  - VXL-3s
  - XL-5
- _Companion computers_:
  - Nvidia Jetson AGX Xavier
  - Nvidia Jetson TX2
- _Autopilots_:
  - PixHawk 6s
- _Chassis sizes_:
  - 1/6th scale
  - 1/10th scale
 
### Software stack
Additionally, OpenConvoy is compatible Ubuntu 18 or greater, supports both ROS 1 and ROS 2, and will run without modification with Python 2 or 3.

**Tested Configurations**:
- ROS 1 Noetic, Ubuntu 18, Python 2
- ROS 2 Foxy, Ubuntu 20, Python 3

## Setup

### Vehicle Setup
