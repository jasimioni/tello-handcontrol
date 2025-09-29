# Setup tello

## Install required packages

```
sudo apt install libasio-dev ros-jazzy-cv-bridge ros-jazzy-camera-calibration-parsers -y
```

## Download and build

```
cd ~/ros2_ws/src
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/jasimioni/tello_ros2_24_04.git
```

Build them:

```
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tello_description tello_msgs tello_driver ros2_shared
```

```
cd ~/ros2_ws
source install/setup.bash
ros2 launch tello_driver teleop_launch.py
```

## Connecting to the tello drone

### WIFI connection

### DHCP and Ping test



