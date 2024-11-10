# How to Run the Installation Scripts

This guide will walk through each step of installing the project dependencies. To ease a complicated dependency installation process, we have written some bash scripts that will install ROS 2 Humble, ArduPilot, SITL, and Gazebo Harmonic automatically. 

> Due to the way dependencies work between ArduPilot, ROS 2, and Gazebo, ***this project and its dependencies can only be installed on an Ubuntu 22.04 system***

## Installation

These instructions assume a freshly-installed Ubuntu 22.04 system.

1. Install Git on the system
    ```
    sudo apt install git -y
    ```
2. Clone this repository
    ``` 
    cd ~
    git clone https://github.com/TEAM-13-VANTAGE/vantage.git
    ```
3. Copy the installation scripts to your home directory and make them executable
    ```
    cp ~/vantage/Docs/scripts/*.sh ~
    chmod +x *.sh
    ```
4. Run the first script (it takes ~30 minutes)
    ```
    ./install_deps1.sh
    ```
    It will ask you to hit the `Enter` key about a minute or two into the script, do it and it won't ask for input again. 

5. After the first script is completed, start SITL **in a new terminal** by running 
    ```
    ./launch_sitl.sh
    ```
6. In **a separate new terminal**, start ROS 2 by running 
    ```
    ./launch_ros2.sh
    ```
7. Once you start receiving data in the ROS 2 terminal, `CTRL+C` both terminals
8. Run the second installation script in one of the new terminals (it takes ~20 minutes)
    ```
    ./install_deps2.sh
    ```
9. After the second script is completed, run the following command **in a new terminal** to start ROS and Gazebo
    ```
    ros2 launch ardupilot_gz_bringup iris_runway.launch.py
    ```