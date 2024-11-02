# Engineering Notebook - Jack Lee

## Before 10/3/2024
- Installed Gazebo and ArduPilot in an Ubuntu 20.04 VM
- Exported the VM and shared it with the team
- Wrote some shell scripts to start the Gazebo and ArduPilot demos
- Helped teammates install Gazebo and ArduPilot
- 

## 10/3/2024
- added .gitignore to this repo to get rid of the .DS_Store files
- Started Engineering Notebook
- Wrote documentation for installing Gazebo and ArduPilot

## 10/15/2024
- Met with team and discussed project status after Hurricane Milton
- Worked with group on the Sprint 2 presentation

## 10/21/2024
- Watched Intelligent Quads youtube videos and followed along to install ROS
  - ROS Foxy is the latest version that supports Ubuntu 20.04, but it's reached end-of-life and is no longer officially supported
  - Catkin, the ROS build tool, has issues working because Foxy is end-of-life
- This gives me a few options: 
  - Keep trying to get it working on Ubuntu 20.04
  - Start over with Ubuntu 18.04, which is what the video tutorials use
  - Start over with Ubuntu 24.04, the newest version of Ubuntu
- There are some more questions I have: 
  - There seems to be two ROS versions: ROS 1 and ROS 2. Which one should we use? 
    - ROS 1 is no longer supported, and is essentially the legacy version (similar to Python before version 3.x.x)
    - ROS 2 will be supported in the future, but it's a lot newer and the tutorials use ROS 1. 
    - ROS 2 is not backward compatible with ROS 1. 

## 10/29/2024
- We agreed to use Ubuntu 22.04 with the following versions of software: 
    - ROS 2 Humble Hawksbill
    - Gazebo Harmonic
    - ArduPilot (the version we were using works with Gazebo Harmonic)
- I will write a script to install these dependencies on a base Ubuntu 22.04 system.

## 10/24/2024
- I created the [script](../Docs/Getting_Started/install_dependencies_22.04.sh) and uploaded it to GitHub under Docs

## 10/26/2024
- Worked on the SRS with Jim and Jona

## 10/29/2024
- Worked on the SRS with everyone else 
- Mai sent me a [robostack](https://robostack.github.io), which lets us use ROS 1 Noetic on Ubuntu 22.04
  - ROS 2 Noetic isn't officially supported on Ubuntu 22.04, so this is a major help
  - Now, we can bridge some of our dependency issues and get Gazebo 11 and ArduPilot working with Ros 1 Noetic
- Worked on the install script to install Ros 1 Noetic instead of Ros 2 Humble

## 11/1/2024
- Investigated Gazebo build error and made a post on Robotics Stack Exchange for help
- Tried the script on Ubuntu 22.04 and 22.04
- no success with building Gazebo 11 so far