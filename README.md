# OpenConvoy: Universal Platform for Real-World Testing of Cooperative Driving Systems
> Pre-print available on [Arxiv](https://arxiv.org/abs/2405.18600)

[Owen Burns](https://owenburns.co), [Hossein Maghsoumi](https://scholar.google.com/citations?user=z-xSxX0AAAAJ&hl=en), [Israel Charles](https://www.linkedin.com/in/israel-charles/), [Yasser Fallah](https://www.ece.ucf.edu/person/yaser-p-fallah/)

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

### Track specification & Leader
OpenConvoy simulates a lead vehicle on your computer according to a pre-set "track" grounded in real-world coordinates. The track contains the straights and turns corresponding to the points on earth you want your vehicles should drive through, as well as the desired speed along each section. Example tracks are located in the ```tracks``` folder.

## Vehicle Setup

### Companion Computer Setup
1. Ensure you can access your companion computer via SSH over Wi-Fi network shared with all other vehicles in the platoon.
2. Install your desired version of ROS. Our code has been tested extensively with [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html).
> **Note**: The setup commands which must be run before starting OpenConvoy assume that this is called catkin_ws (ROS1) or ros2_ws (ROS2). Please modify those commands if using a different workspace name
3. Install [mavros](https://github.com/mavlink/mavros/blob/master/mavros/README.md)
4. Install dependencies
```
pip install -r requirements.txt
```

### Autopilot Setup
1. Assemble the vehicle, ensuring that the autopilot is connected to power and the ESCs are connected to the autopilot. If present, remove the red +5v wire from the ESC and feed it into the +5v wire coming from the servo. This is often necessary if the servo is designed to receive power from the ESC. If not, still remove the +5v wire from the ESC but cap it instead of plugging it into the autopilot.
2. Using the ground control station of your choice, set up the parameters necessary to control your vehicle from the ground control station. We used QGroundcontrol.
3. Change the settings necessary to enable commands from a companion computer. This step differs in implementation depending on whether you use [Ardupilot](https://ardupilot.org/dev/docs/companion-computers.html) or [PX4](https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html).

## Leader setup
The lead vehicle of the platoon is simulated by your computer in order to set the driving pace and maximize the number of cars generating meaningful data about the effectiveness of your cooperative driving algorithm. 

1. Install dependencies
```
pip install -r requirements.txt
```

## Running OpenConvoy
First, ensure that all of the vehicles in the convoy (as well as the computer which will be running the leader code) are on the same Wi-Fi network. Then, on each of the vehicles in your convoy, run the following commands depending on the ROS version used

### ROS 1
```
source catkin_ws/devel/setup.bash
roscore > roscore_out.txt 2>&1 &
sudo chmod 777 /dev/ttyS0
rosrun mavros mavros_node _fcu_url:=/dev/ttyS0:115200 > mavros_out.txt 2>&1 &
rosservice call /mavros/set_stream_rate '
stream_id: 0
message_rate: 20
on_off: 1'
python [YOUR MISSION FILE] ros1 --track_path=[YOUR TRACK] --save_path=[YOUR SAVE LOCATION] --car_number=[CAR POSITION IN CONVOY]
```

### ROS 2
```
source ros2_ws/install/setup.bash
sudo chmod 777 /dev/ttyTHS0
ros2 run mavros mavros_node --ros-args --param fcu_url:=/dev/ttyTHS0:921600 > mavros_out.txt 2>&1 &
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 20, on_off: 1}"
python [YOUR MISSION FILE] ros2 --track_path=[YOUR TRACK] --save_path=[YOUR SAVE LOCATION] --car_number=[CAR POSITION IN CONVOY]
```

Finally, run the following commands on your computer to start the leader and move the vehicles:
```
python utils/beacon_broadcase.py --track_path=[YOUR TRACK] --broadcast_int=[YOUR BROADCAST INTERVAL] --drop_rate=[OPTIONAL, how frequently to skip sending a message to simulate communication issues]
```
