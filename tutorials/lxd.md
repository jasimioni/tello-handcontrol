# Using LXD with ROS

## Setup LXD

https://documentation.ubuntu.com/lxd/stable-5.21/tutorial/first_steps/

## LXD Container

Initialize a container with the required Ubuntu version. For ROS Jazzy, Ubuntu is 24.04, so:

```
lxc init ubuntu:24.04 ros-jazzy 
```

## Devices

You can passthrough some devices to be used by the container. The ones I'll recommend here:

### GPU

```
lxc config device add ros-jazzy gpu gpu
```

### Webcam (for mediapipe)

For webcam you need to pass the video device associated to the camera (`/dev/video*`)

To see which camera is attached to each dev, use:

```
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

In my case:

```
Brio 500 (usb-0000:00:14.0-4):
        /dev/video0
        /dev/video1
        /dev/media0
```

So I'll pass /dev/video0 and /dev/video1. The UID 1000 is to make the ubuntu user the owner.

```
lxc config device add ros-jazzy video0 unix-char path=/dev/video0 uid=1000
lxc config device add ros-jazzy video1 unix-char path=/dev/video1 uid=1000
```

### Wifi Adapter

For tello, the container needs to have the IP address taken from the tello network, so passing the NIC
to the container for full control. To check the nic names use:

```
ip link
```

And to pass the NIC to the container, use:

```
lxc config device add ros-jazzy wlan0 nic nictype=physical parent=wlp0s20f3
```

The NIC will vanish from the host when the container is up.

### Starting and accessing the container

```
lxc start ros-jazzy
```

```
lxc list
```

```
+-----------+---------+----------------------+-----------------------------------------------+-----------+-----------+
|   NAME    |  STATE  |         IPV4         |                     IPV6                      |   TYPE    | SNAPSHOTS |
+-----------+---------+----------------------+-----------------------------------------------+-----------+-----------+
| ros-jazzy | RUNNING | 10.236.89.253 (eth0) | fd42:4c75:135b:16cc:216:3eff:fec5:d2af (eth0) | CONTAINER | 0         |
+-----------+---------+----------------------+-----------------------------------------------+-----------+-----------+
```

The shell command will allow you to access it. From the shell, you can add your keys to the ubuntu
user and use ssh to connect to it.

```
lxc shell ros-jazzy
```


### SSH Configuration

Edit your SSH configuration and add the following options:

```
vim ~/.ssh/config
```

```
Host ros-jazzy
    User ubuntu
    ForwardAgent yes
    ForwardX11 yes
```

Add an entry to your /etc/hosts with the ip address from the lxc list

```
sudo vim /etc/hosts
```

```
10.236.89.253   ros-jazzy
```

The `ForwardX11` option will allow you to run rviz2 from the container and display in your desktop.