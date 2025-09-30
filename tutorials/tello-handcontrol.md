# This is the real code to control the drone

## Download the source

```
cd ~/ros2_ws/src
git clone https://github.com/jasimioni/tello-handcontrol.git
```

## Build the package

```
cd ~/ros2_ws/
colcon build --packages-select tello_mediapipe
```

## Run it

```
cd ~/ros2_ws
source install/setup.bash
ros2 run tello_mediapipe gestures
```
