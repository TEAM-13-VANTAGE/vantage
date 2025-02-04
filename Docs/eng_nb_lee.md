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

## 10/22/2024
- We agreed to use Ubuntu 22.04 with the following versions of software: 
    - ROS 2 Humble Hawksbill
    - Gazebo Harmonic
    - ArduPilot (the version we were using works with Gazebo Harmonic)
- I will write a script to install these dependencies on a base Ubuntu 22.04 system.

## 10/24/2024
- I created the [script](scripts/install_deps1.sh) and uploaded it to GitHub under Docs

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

## 11/4/2024
 - Interviewed Dr. Akbas with Jona and wrote a transcript of questions and answers
 - Researched tutorials and resources for using modern versions of Gazebo, Ubuntu, and ROS 2

## 11/5/2024
 - Worked on SDD
 - <img alt="AAAAAAAAHHHHHHH" src="https://i.pinimg.com/originals/f5/ef/50/f5ef5000641ee61a1866e04430db71fd.gif">

## 11/7/2024
 - Installed Ubuntu 24.04 and started making a script to install Gazebo Harmonic and ROS 2 Jazzy
 - Realized ArduPilot isn't compatible with 24.04 because it only works with Ros 2 Humble
 - Restarted on Ubuntu 22.04 and made four scripts to install everything
 - They worked for me, so I started a fresh VM to test them
 - I'll keep remaking a fresh VM and testing the scripts until they work first try from scratch
 - I also want to determine how long it takes to install everything and how much storage it takes
 - I'll do all that tomorrow

 ## 11/9/2024
 - Tested the installation using my scripts and fixed a couple of typos and convenience issues
 - It took 1hr and 3 minutes the first time, and 56 minutes the second time
 - Both installations took about 11Gb of storage
 - I'll document how to run the scripts tomorrow, but it's self explanatory if anyone wants to try it themselves

 ## 11/10/2024
 - wrote documentation for running the installation scripts

 ## 11/20/2024
 - worked on creating graphics for the Poster v1
 - wrote Methodology section for Poster v1
 - helped arrange elements on Poster v1

## 11/25/2024
 - Read SRS v1 feedback
 - Removed sections of the SRS that were not applicable for our project
 - Modified wording to address feedback in sections 1.1, 1.5.2, 2.1, 2.7, and 4.3

## 12/1/2024
 - Started 10 minute video slides
 - Worked on script for the video following the rubric

## 12/2/2024
 - Worked on script and slides for 10 minute video with Jona

## 12/3/2024
 - Completed script for 10 minute video
 - Arranged and created slides to follow the script
 - Bought poster board for groups 13, 8, and 1

## 12/4/2024
 - Worked on sections of the STP
 - Recorded slides for the 10-minute video presentation

## 12/5/2024
 - Presented with the other groups in Lehman Atrium
 - Dr. T gave me a cookie :D
 - Mostly positive feedback, Dr. Akbas mentioned to emphasize the open-source nature in the future

## 1/14/2024
 - Added Jira tasks for Environmental, Economic, Social, and Cultural Effects
    - I will work on the Economic Effects section
 - Met with team to discuss future work with the project
    - Mai and I will work on the Julia side, Jim and Jona are switching to the Gazebo side

## 1/16/2025
 - Worked on getting Julia to work for my computer
 - Gazebo is now working with two drones!!!

## 1/21/2025
 - Worked more on getting Julia to work on my computer
 - Contacted John to help, he will meet us on Thursday

## 1/23/2025
 - Worked with John to get Julia working on my computer
 - Julia works now! 

## 1/28/2025
 - Helped Mai get Julia working on her computer
 - Started working on head-on collisions in Julia

 ## 1/30/2025
 - Started running simulations in Julia
 - Troubleshot head-on collisions
 - Added functionality to the Julia code so that simulations can run on any computer
   - I made it use relative instead of absolute paths and it now checks if the csv files exist and creates them if not

## 2/2/2025
 - Worked on SRS
 - Created Section 3 and wrote 3.1, 3.2, 3.3 to describe how our project will affect Public Health, Safety, and Welfare
 - Created and wrote section 3.8 to describe the Economic Effects of our project
 - Updated document control table and table of contents

## 2/4/2025
 - Messed around with the visualization and graph generation scripts in `Particle-Simulation\analysis\headon-visualize.ipynb`. 
 - Made an example script to plot the results of headon-horizontal simulations
 - Added section 4.4 to SRS
 - Future work: Find where the UI saves simulations to, and add an option / feature to display the result graphs to the user after simulation execution.