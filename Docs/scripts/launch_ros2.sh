#!/bin/bash

# A simple script to launch ROS 2
# Run `launch_sitl.sh` first for this to work
source ~/ardu_ws/install/setup.bash
ros2 node list
ros2 node info /ardupilot_dds
ros2 topic echo /ap/geopose/filtered