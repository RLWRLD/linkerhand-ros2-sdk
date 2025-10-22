# Linkerhand ROS2 SDK

## Overview
ROS2 SDK to utilize Linkerhand L20 @ RLWRLD 

## Installation  
Tested on ubuntu 22.04, ROS2 humble, python 3.10.12

```bash
# Install dependencies
sudo apt install python3-can
pip install -r requirements.txt

# Build
colcon build --symlink-install
```  

## Usage
```bash
# CAN interface name can be different, check by "ifconfig -a" command.
sudo ip link set can0 up type can bitrate 1000000 # This is needed only once after booting.
source install/setup.bash  

### The below command are to run ros2 nodes.
### Modify parameters depend on your use cases.

# 1. Node to control L20.
ros2 run linker_hand_ros2_sdk linker_hand_sdk --ros-args -p hand_type:=right -p hand_joint:=L20 -p is_touch:=True -p can:=can0

# 2. GUI control panel
ros2 run gui_control gui_control --ros-args -p hand_type:=right -p hand_joint:=L20 -p topic_hz:=30 -p is_touch:=True -p is_arc:=False

# 3. Tactile sensor visualizer
ros2 run matrix_touch_gui matrix_touch_gui --ros-args -p hand_type:=right -p hand_joint:=L20
```