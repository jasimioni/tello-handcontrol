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

You need to directly connect to the drone from the container to get an IP address and be able to
receive the return data. From the container, install iw:

```
sudo apt install iw
```

Then run the scan:

```
sudo iw dev eth1 scan | grep TELLO
```

Get the SSID name 

```
        SSID: TELLO-CFCEAB
```

Create a file named /etc/wpa_supplicant.conf with the content:

```
network={
    ssid="TELLO-CFCEAB"
    key_mgmt=NONE
}
```

And connect to it using:

```
sudo wpa_supplicant -i eth1 -c /etc/wpa_supplicant/wpa_supplicant.conf
```

Everything is good when you see:

```
eth1: Associated with 60:60:1f:cf:ce:ab
eth1: CTRL-EVENT-CONNECTED - Connection to 60:60:1f:cf:ce:ab completed [id=0 id_str=]
eth1: CTRL-EVENT-SUBNET-STATUS-UPDATE status=0
eth1: CTRL-EVENT-SIGNAL-CHANGE above=1 signal=-39 noise=9999 txrate=18000
```

### DHCP and Ping test

Install the dhcp client:

```
sudo apt install isc-dhcp-client
```

And with the wpa_supplicant connected, run:

```
sudo dhclient eth1
```

Remove the route created:

```
sudo ip route del default via 192.168.10.1
```

You can check if connectivity is good using:

```
ping 192.168.10.1
```

```
PING 192.168.10.1 (192.168.10.1) 56(84) bytes of data.
64 bytes from 192.168.10.1: icmp_seq=1 ttl=255 time=2.81 ms
--- 192.168.10.1 ping statistics ---
1 packets transmitted, 1 received, 0% packet loss, time 0ms
rtt min/avg/max/mdev = 2.812/2.812/2.812/0.000 ms
```

## Sending commands to tello

```
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
```

```
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```