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
- Updated and submitted SRS
- Future work:
  - Replace the test.jl running in gui.py with the round1 simulations
  - Display the result graphs to the user after simulation execution.
  - Add the option to import existing simulation parameters

## 2/6/2025

- discussed peer reviews with Dr. Akbas and team
- Worked on getting julia output to appear in the gui
  - no success so far -- We would need another process to check for the julia stdout and return it as it's printed

## 2/11/2025

- Added a button to import .csv parameter files
  - Started working on parsing the files into simulation parameters

## 2/17/2025

- Simulation CSV files can now be imported to the GUI!

## 2/18/2025

- Discussed new sprint tasks and our progress last sprint
- I will be working on rendering and displaying the graphs after simulation completion

## 2/20/2025

- Had a work meeting so I was late to the class
- Discussed the future roadmap and plans to complete the project this semester

## 2/25/2025

- Discussed Julia-python integration with team
- Worked on running Gazebo sim in Ubuntu 22.04

## 2/27/2025

- Worked on integrating Julia and Python further
  - Julia now reads the `params.csv` outputted by gui.py
- Considered working on running Gazebo sims from a Docker container in Ubuntu 22.04
    - Docker containers don't usually have graphics, so this would be difficult without some hacky workarounds
- Future work: Run visualization after simulation completion in python gui

## 3/3/2025

- Fixed and pushed updated `round1-full.jl` and `search.jl`
- Started working on displaying result visualizations after simulation completion

## 3/6/2025

- Discussed upgrading from Ubuntu 20.04 to 22.04 to get Gazebo working
- Tested the current Julia and Python code on an Ubuntu 22.04 VM, it works

## 3/11/2025

- Spring Break
- Represented Keysight at the US Cyber Trust Mark technical requirements committee meeting
    - See: https://www.fcc.gov/CyberTrustMark

## 3/13/2025

- Spring Break
- Had a job interview for a Keysight R&D position

## 3/18/2025

- Regrouped from break, finalized the transition from Ubuntu 20.04 to 22.04
- Ran Jim's excellent installation script with his documentation, troubleshot a small error in class
- Discussed a display of each low-fidelity simulation for users to investigate and run in high-fidelity

## 3/20/2025

- Worked on the simulation visualizations, goal is to finish it this week
- The result data isn't in the correct format for the graphs to display, but I don't know what format it wants

## 3/25/2025

- Was out of town and drove back today
- I think I'll put the visualizations aside and work on the result display feature Jim talked about in class

### 3/26/2025

- Merged the existing code to visualize the results to main
- Started working on the result display
- Finished the results table display :D

## 3/27/2025

- Showed results table to the team and elicited feedback


## 4/1/2025

- Discussed the remaining time left with the team and planned the rest of the timeline
- We will focus on implementation and completing the project by the end of the semester
- We did an April Fools joke on Akbas that we had to start over again lol

## 4/3/2025

- Jim mentioend that there should be a check box on each row the user can select
  - the user will be able to check the boxes on rows they want to explore further and then run them in high-fidelity
- I started researching ways of adding the check boxes

## 4/8/2025

- Started implementing the checkboxes, they work so far
- Next steps: Handle the checked rows by adding their contents to a list of high-fidelity parameters

## 4/9/2025

- Finished check boxes and pushed to GitHub
- When a box is checked, the row's data is added to a 2-D list of sim parameters
- When a box is unchecked, that row's data is removed from the list
- When we run the high-fidelity sim, we can use the list of parameters created from the check boxes

## 4/10/2025

- Group showcased the GUI to Dr. Akbas and Alejandro
- Discussed timeline, we should finish the project by next week
- I will work on ensuring SI unit consistencey and clean up the code
- Sent John a discord message to ask what units Julia uses

## 4/12/2025

- John says that Julia uses Imperial units, so I will have to convert them for Gazebo, which uses SI units. 
  - I plan to change the UI to only use SI units and do unit conversion when data is moved to and from Julia
  - This could potentially lose some precision, but precision would be lost regardless with a unit conversion. All units in the JuliaSim already use 6 significant figures of precision so I'll follow that
- Some scratch work: 

    ```rs
    1mi = 5280ft
    1h = 3600s
    1fps = 1ft/1s
    1mph = 1m/1h
    1mph = 5280ft/3600s = 1.46667fps

    1m = 3.28084ft
    1h = 3600s
    1km = 1000m = 3280.84ft
    1fps = 1ft/1s
    1km/h = 1ft/1s * 1km/3280.84ft * 3600s/1h = 1.09728fps
    1m/s = 1ft/1s * 1m/3.28084ft * 1s/1s = 0.304800fps
  ```

- I also updated the way unit drop-down boxes are created for the UI so that only relevant units for each simulation parameter appear in each drop-down box