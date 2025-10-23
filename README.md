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

# Launch L20 to send position target via ROS2 msg
ros2 launch linker_hand_ros2_sdk linker_hand_radian_cmd.launch.py source:=ros
# Or via ZMQ
ros2 launch linker_hand_ros2_sdk linker_hand_radian_cmd.launch.py source:=zmq
```

## How to Send Command
*TODO(SH)*: Check left hand usability!

### 1. By ROS2 topic
#### `/linkerhand_right/reference`, `linkerhand_left/reference` (`sensor_msgs/JointState`) 
- position commands in radian.
- Joint order:
    - [thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch,

        thumb_cmc_roll, index_mcp_roll, middle_mcp_roll, ring_mcp_roll, pinky_mcp_roll,
        
        thumb_cmc_yaw, NAN, NAN, NAN, NAN,
        
        thumb_mcp, index_pip, middle_pip, ring_pip, pinky_pip] 
    - running gui_control node might be helpful to check joint order.


```bash
# Example command in CLI. 
ros2 topic pub --once /linkerhand_right/reference sensor_msgs/msg/JointState \
    "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 2. By ZMQ
Please check the code block to send command through zmq.

More details are in `linker_hand_ros2_sdk/test_zmq_publisher.py`

```python
import zmq
import struct

endpoint = "ipc:///tmp/linkerhand_reference"
context = zmq.Context()
sock = socket.socket(zmq.PUB)
sock.bind(endpoint)

hand = "left" # or "right"
data = [0.0 for _ in range(20)] # list(float)
payload = struct.pack("<20f", *data)
sock.send_multipart([hand.encode("utf-8"), payload], flags=zmq.NOBLOCK)
```

# Node description
Please note that you may need to change parameters.
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