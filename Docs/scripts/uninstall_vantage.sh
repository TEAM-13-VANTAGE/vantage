#!/bin/bash

# VANTAGE Project Uninstallation Script

# Uninstall ROS 2 Humble
echo "# Removing ROS 2 Humble #"
sudo apt remove --purge ros-humble-desktop ros-dev-tools -y
sudo apt autoremove --purge -y
sudo apt clean
echo "Removed ROS 2 Humble."

# Remove ROS 2 repository
echo "# Removing ROS 2 repository #"
sudo rm /etc/apt/sources.list.d/ros2.list
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
echo "Removed ROS 2 repository."

# Remove rosdep initialization
echo "# Removing rosdep initialization #"
sudo rm -rf /etc/ros/rosdep/sources.list.d
sudo rm -f ~/.rosdep
echo "Removed rosdep initialization."

# Uninstall ArduPilot
echo "# Removing ArduPilot installation #"
cd ~/ardu_ws
sudo apt remove --purge default-jre python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
sudo apt autoremove --purge -y
sudo apt clean

# Remove Micro-XRCE-DDS-Gen
echo "# Removing Micro-XRCE-DDS-Gen #"
cd ~/ardu_ws/Micro-XRCE-DDS-Gen
./gradlew clean
cd ~/ardu_ws
sudo rm -rf Micro-XRCE-DDS-Gen
echo "Removed Micro-XRCE-DDS-Gen."

# Remove the workspace folder
echo "# Removing ArduPilot workspace #"
cd ~
sudo rm -rf ~/ardu_ws
echo "Removed ArduPilot workspace."

# Remove environment variable changes
echo "# Removing environment variable changes #"
sed -i '/\/opt\/ros\/humble\/setup.bash/d' ~/.bashrc
sed -i '/\/ardu_ws\/install\/setup.bash/d' ~/.bashrc
sed -i '/microxrceddsgen/d' ~/.bashrc
echo "Removed environment variable changes."

# Clean pip packages
echo "# Removing pip packages #"
pip3 uninstall -y PyYAML mavproxy pymavlink
echo "Removed pip packages."

echo "# VANTAGE Project Uninstallation Complete #"

