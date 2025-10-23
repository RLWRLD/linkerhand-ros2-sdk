# Linkerhand ROS2 SDK
Linkerhand L20 ROS2 SDK @ RLWRLD 

# Installation  
Tested on ubuntu 22.04, ROS2 humble, python 3.10.12

```bash
# Install dependencies
sudo apt install python3-can
# pip install -r requirements.txt

# Build
colcon build --symlink-install
```  

# Usage
## Simple usage
```bash
# CAN interface name can be different, check by "ifconfig -a" command.
sudo ip link set can0 type can bitrate 1000000 # This is needed only once after booting.

# Source the package
source install/setup.bash  

# Launch L20 and position command encoder(radian -> byte)
ros2 launch linker_hand_ros2_sdk linker_hand_radian_cmd.launch.py
```

## Topic description
### `/linkerhand_right/reference`, `linkerhand_left/reference` (`sensor_msgs/JointState`)
- position commands in radian.
- Joint order:
    - [thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch,

        thumb_cmc_roll, index_mcp_roll, middle_mcp_roll, ring_mcp_roll, pinky_mcp_roll,
        
        thumb_cmc_yaw, NAN, NAN, NAN, NAN,
        
        thumb_mcp, index_pip, middle_pip, ring_pip, pinky_pip] 
    - running gui_control node might be helpful to check joint order.
- *TODO(SH)*: Check left usability!

```bash
# Example command in CLI. 
ros2 topic pub --once /linkerhand_right/reference sensor_msgs/msg/JointState \
    "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

# Node description
Please note that you may change parameters.
### Main node to run L20
```bash
ros2 run linker_hand_ros2_sdk linker_hand_sdk --ros-args -p hand_type:=right -p is_touch:=True -p can:=can0
```

### GUI control panel
```bash
ros2 run gui_control gui_control --ros-args -p hand_type:=right -p topic_hz:=30 -p is_touch:=True -p is_arc:=False
```

### Tactile sensor visualizer
```bash
ros2 run matrix_touch_gui matrix_touch_gui --ros-args -p hand_type:=right
```