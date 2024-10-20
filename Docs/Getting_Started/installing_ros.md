# Installing ROS 2 Foxy on Ubuntu 20.04

## Reference used:https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
## Step 1: Set up Your System

1. **Open a Terminal**
2. **Update your system** :
   ```bash
   sudo apt update
   sudo apt upgrade

3. Install necessary tools:

## Step 2: Add the ROS 2 Repository
1. **Add the ROS 2 key**:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
```
2. **Add the ROS 2 repository**:
```bash
echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list
```
## Step 3: Install ROS 2 Foxy:
1. **Update your package index**:
```bash
sudo apt update
```
2. **Install ROS 2 Foxy**:
```bash
sudo apt install ros-foxy-desktop
```

## Step 4: Initialize rosdep

1. **Install rosdep**:
```bash
sudo apt install pyhton3-rosdep
```
2. **initialize rosdep**:
```bash
sudo rosdep init
rosdep update
```

## Step 5: Environment Setup
1. **Source the ROS setup file:**
```bash
source /opt/ros.foxy/setup.bash
```
To make this permanent, add the following line:
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 6: Install additional Tools
```bash
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Step 7 Verify the Installation
1. **Run a simple ROES 2 command**
```bash
ros2 run demo_nodes_cpp talker
```


