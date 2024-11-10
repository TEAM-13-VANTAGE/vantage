#!/bin/bash

# VANTAGE Project Installation Script 1

# Install ROS 2 Humble
echo "# Installing ROS 2 Humble #"
sudo apt install software-properties-common git cmake curl -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -y
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo rosdep init

# Install ArduPilot
echo "# Installing ArduPilot #"
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
cd ~/ardu_ws
sudo apt update -y
rosdep update
# source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install default-jre -y
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=$PATH:$PWD/scripts" >> ~/.bashrc
source ~/.bashrc
export PATH=$PATH:$PWD/scripts
microxrceddsgen -help
sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
pip3 install PyYAML mavproxy --user
pip3 install mavproxy pymavlink --user --upgrade
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests

# Test ArduPilot ROS 2
# echo "# Testing ArduPilot and ROS 2 installation #"
cd ~/ardu_ws
source ./install/setup.bash
echo "source ~/ardu_ws/install/setup.bash" >> ~/.bashrc
# colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
# colcon test-result --all --verbose

# Install SITL
echo "# Installing SITL #"
# source /opt/ros/humble/setup.bash
cd ~/ardu_ws/
colcon build --packages-up-to ardupilot_sitl
# source install/setup.bash
echo "# ROS 2 Humble, ArduPilot, and SITL are now installed #"
echo "# Perform the following steps to continue with the dependency installation #"
echo "# Start SITL by running 'launch_sitl.sh' in one terminal #"
echo "# Start ROS 2 by running 'launch_ros2.sh' in another terminal #"
echo "# Once you start recieving data in the ROS 2 terminal, CTRL+C both and run 'install_deps2.sh' #"


