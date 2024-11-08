#!/bin/bash

# VANTAGE Project Installation Script 2

# Install Gazebo
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

# Set Up ROS 2 Packages
echo "# Setting up ROS 2 packages #"
cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
export GZ_VERSION=harmonic
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r

# Build ArduPilot Plugin for Gazebo
echo "# Building ArduPilot Plugin for Gazebo #"
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup

# Test Installation
echo "# Testing Installation #"
cd ~/ardu_ws
source install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose

# Launch Gazebo
echo "# The installation is complete! #"
echo "# Run the following command to launch Gazebo: #"
echo "# ros2 launch ardupilot_gz_bringup #" iris_runway.launch.py