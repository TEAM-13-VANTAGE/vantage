#!/bin/bash

# This script will install the dependencies for the project 
# (ROS 1 Noetic, Gazebo 11, ArduPilot, ArduPilot Plugin for Gazebo)
# It shoud be used on a base Ubuntu 22.04 system. It is untested elsewhere. 

# install build dependencies
echo "# Installing build dependencies #"
sudo apt-get install curl git g++ cmake -y

# install conda and mamba
echo "# Installing Conda and Mamba #"
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh -b -p "${HOME}/conda"
source "${HOME}/conda/etc/profile.d/conda.sh"
source "${HOME}/conda/etc/profile.d/mamba.sh"
echo 'source $HOME/conda/etc/profile.d/conda.sh' >> ~/.bashrc
echo 'source $HOME/conda/etc/profile.d/mamba.sh' >> ~/.bashrc
conda activate
conda install mamba -c conda-forge

# install ROS noetic
echo "# Installing ROS Noetic #"
mamba create -n ros_env python=3.11
mamba activate ros_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
mamba install ros-noetic-desktop
mamba deactivate
mamba activate ros_env
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep

# install Catkin and setup workspace
echo "# Installing Catkin #"
#sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-pkg python3-catkin-pkg-modules
cd ~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws
wstool init ~/catkin_ws/src
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
cd ~/catkin_ws
catkin build
source ~/.bashrc

# install ArduPilot
echo "# Installing ArduPilot #"
cd ~
git clone https://github.com/ardupilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
git config --global url.https://.insteadOf git://
git checkout Copter-4.0.4
git submodule update --init --recursive
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w

# install Gazebo 11
echo "# Installing Gazebo 11 #"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update -y
sudo apt-get install gazebo11 libgazebo11-dev
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc

echo "# Reboot your computer for SITL to work #"
echo "# use \"mamba activate ros_env\" to use ROS Noetic #"
echo 'echo "# use \"mamba activate ros_env\" to use ROS Noetic #"' >> ~/.bashrc