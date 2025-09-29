# Setup USB Camera

## Confirm webcam is working:

Use the official ROS2 docs to install a webcam node:

https://docs.ros.org/en/jazzy/p/usb_cam/


- Install python library pydantic:

```
sudo apt install python3-pydantic
```

Install the ROS2 module:

```
sudo apt-get install ros-jazzy-usb-cam -y
```

Launch it:

```
ros2 launch usb_cam camera.launch.py
```

Check if the topic is publishing:

```
ros2 topic list
```

```
/camera1/camera_info
/camera1/compressedDepth
/camera1/image_compressed
/camera1/image_raw
/camera1/image_raw/theora
/image_raw/zstd
/parameter_events
/rosout
```

Confirm you can subscribe to it using rviz2:

```
rviz2
```

Go to add -> By topic -> /camera1/image_raw