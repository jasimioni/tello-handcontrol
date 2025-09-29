# Setting up mediapipe

## Media PIPE

Using base instructions from:

https://github.com/dmartinelli1997/media_pipe_ros2

- Install opencv and get the git repository

```
sudo apt install python3-opencv
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/dmartinelli1997/media_pipe_ros2
```

- Build the package

``` 
cd ~/ros2_ws
colcon build --packages-select media_pipe_ros2 media_pip_ros2_msg
```

- Install the python dependencies. ROS2 seems to ignore venvs, so installing system wide:

```
sudo pip install mediapipe --target /usr/local/lib/python3.12/dist-packages
```

- Run it

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run media_pipe_ros2 hands_detector
```
