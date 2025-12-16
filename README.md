# jq_tactile_driver

ROS 2 driver for the JQ Precision Textile Electronic Skin (tactile suit).

Tested on Ubuntu 22.04 with ROS 2 Humble.

## Install

In your ros2 workspace, build the package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/schefferac2020/tq_tactile_driver.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run

```bash
ros2 run jq_tactile_driver serial_node --ros-args -p port:=/dev/ttyACM0
```
