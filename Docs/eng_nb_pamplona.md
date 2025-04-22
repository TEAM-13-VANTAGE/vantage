# Jim's Engineering Journal

## Sprint 1
### Summary
During sprint 1, we got familiar with the project scope and looked into the different technologies associated with the project. We are using Julia for the low-fidelity simulation and Gazebo and ArduPilot for the high-fidelity. As I am on a Mac, I installed a VMware in order to set up the programs needed. I then set up Gazebo and ArduPilot to ensure that they are integrated and are communicating properly. 

## Sprint 2
### 10/1/2024
- VMware broke, so I had to fix the installation and integration of Gazebo and ArduPilot.
- Did research on ways we could make the VMware run Gazebo faster, didn't get anywhere.
- Started new sprint and allocated tasks to everyone.
- Started researching on Julia and how to create python frontend for simulation controls.
### 10/3/2024
- Installed Julia on the VMware (It was really easy, I just had to curl it).
- Did more research on python frontend.
- Reviewed GitHub repo with previous documentation that Alejandro shared.
- Created GitHub engineering journal.
### Summary
Was not able to work due to hurricane and me being at the SASE National Convention. Other members had no power due to the hurricane.

## Sprint 3
### 10/15/2024
- VMware broke again, so I will just install Linux, Ubuntu, and Gazebo on my PC, while working with Julia on Mac.
- Moved all sprint 2 stories and tasks to sprint 3.
- Working on slides to summarize what we have already completed.
### 10/21/2024
- Got JuliaSim to run!
<img width="300" alt="image" src="https://github.com/user-attachments/assets/db59acc2-4d9f-4847-923d-39e15bae1a11">

- Beginning frontend work on Python using Flask and basic HTML.
<img width="300" alt="image" src="https://github.com/user-attachments/assets/3029b1c1-1d01-452c-bedf-0acd4a5c3ce2">

### 10/24/2024
- Converting from Flask and HTML to pure Python.
- Attempted to run the preexisting sims created by John, and it said it'll finish in four days...
<img width="214" alt="image" src="https://github.com/user-attachments/assets/4336f4a1-4e1e-422e-8c76-982e21469de7">
- Assisting teammates with their ROS issue.
- Starting SRS v1.

## Sprint 4
### 10/29/2024
- Completed the Use Case, Scenarios, and DFDs for the SRS v1.
<img width="343" alt="image" src="https://github.com/user-attachments/assets/9d9e2089-a5dc-4a61-83a7-30c416c78e07">
<img width="480" alt="image" src="https://github.com/user-attachments/assets/02bb521d-278e-4874-9032-42729a453e08">
<img width="494" alt="image" src="https://github.com/user-attachments/assets/349256b3-66d0-4bbe-95ec-0585f1e3e7cf">

- Submitted SRS v1.
- Did our stand up to begin the next sprint.
### 10/31/2024
- Beginning SDS v1.
- Working on researching more on python front-end.
- Coordinating interview with Dr. Akbas and Alejandro

### 11/05/2024
- Created questions for interview with Akbas.
- Completed SDD v1 sections 4 and 5.
- Assisting team members for SDD.
- Assisting members for ROS and Gazebo integration.

### 11/07/2024
- Completed Small update slides
- Ended sprint!

## Sprint 5
### 11/12/2024 
- Updated small update slides and presented
- Created prototype of Front-end on Figma
- https://www.figma.com/design/3LFUXZGglFq9kiKeomucW9/VANTAGE?node-id=0-1&t=2cAi4HDekWI6eR28-1
- ROS, Gazebo, and Ardupilot up and running on PC WOOHOOO!!!!

### 11/14/2024
- Not much updates, was out of class due to sickness
- Did more during the weekend, began Python frontend. 

### 11/19/2024
- Python frontend draft fianlly working for Julia!
<img width="800" alt="image" src="https://github.com/user-attachments/assets/e69b728e-b29f-42f5-8e05-ad501ba23a1a">

- Testing simulations for Julia and trying to understand how it works.

### 11/21/2024
- Met with John to understand the results
- Was able to run simulations in parallel! Ran very fast :)
- Completed SDD and Poster

### 11/26/2024
- Completed SRS v2
- Talked with Dr. Akbas and Alejandro regarding future of project

## Sprint 6
### 12/3/2024
- Worked on 10-minute Video presentation
- Finalized Poster v2
- Started researching on Drone Swarming, still no progress as all videos/documentation are outdated.
- Got LiDAR Working!
<img width="800" alt="image" src="https://github.com/user-attachments/assets/753248d6-76df-49e9-8533-c3136d62a65e">

--------------
## Sprint 1
### 1/9/2025
- Welcome back!
- Talked to Akbas and Alejandro about smeester expectations

### 1/14/2025
- Reworked backlog to be more atomic
- Set new plan for semester and sprint
- Found a new repo to utilize for Gazebo

### 1/16/2025
- Helped Jack and Mai with some Julia stuff
- Successfully set up older version of Gazebo and ardupilot in Jona's machine
- Successfully installed it on my pc


## Sprint 2
### 1/21/2025
- Implemented LiDAR to Gazebo
- Got both 2-drone swarms and LiDAR to work together in one sim
<img width="800" alt="image" src="https://github.com/user-attachments/assets/73698681-6190-4ba2-a8dc-fc4849d28a73">


### 1/23/2025
- Helped Jona look for ways to control drones using CLI

### 1/28/2025
- Sick so was unable to go

### 1/30/2025
- Working on SRS

### 2/4/2025
- Talked with PO and TA
- Scrum standup meeting
- Ended sprint and talked about next steps

## Sprint 3
### 2/6/2025
- Began sprint
- Helped Jack and Mai on low-fidelity module
- Helped Jona with Waypoint system

### 2/11/2025
- Assisted Jona again with waypoint system
- Could not seem to get it to work
- Starting to rethink approach for movement within the high-fidelity module

### 2/13/2025
- Started working on integrating the high-fidelity module to the controller
- Many issues arised as the script was not set up on my pc
- Awaiting for commands needed for waypoint system
- Reverting back to command approach for movement in the high-fidelity module
  - https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

### 2/18/2025
- Ended sprint and talked about next steps
- Scrum standup meeting
- Completed integration of high-fidelity module within the controller
 - A lot of issues that need to be considered
   - Ardupilot not running properly; can't find mavproxy
   - System limited to Python 3.8 due to Ubuntu 20.04
   - Python controller breaking due to older version

## Sprint 4
### 2/20/2025
- Began sprint
- Started researching on upgrading to Ubuntu 22.04
- Scrapped all that was done on Ubuntu 20.04

### 2/25/2025
- Installed Ubuntu 22.04 on pc

### 2/27/2025
- Was able to install ROS2, Gazebo, and Ardupilot smoothly on VM
- Starting research again on how to implement lidar + multidrone

### 3/04/2025
- Working on more research
- Assisted Jack with some issues on the Julia and Python side
- Worked on SDD

## Sprint 5
### 3/18/2025
- Started new sprint
- Got LiDAR + Multidrone working on Ubuntu 22.04!
- Made plan for rest of semester
- Created installation guide for the system on a markdown file

### 3/20/2025
- Assisted everyone with installation for the system

### 3/25/2025
- Started research on object avoidance implementations
- Started Python script for parameter conversion

### 3/27/2025
- Not much, getting cooked by other classes :,)

## Sprint 6
### 4/1/2025
- Worked on drone commands script
- Was having difficulties due to finnicky depenedencies (Again)

### 4/3/2025
- Debugged drone commands script as they were not pinging the drones
- Finalized translating the parameters to high-fidelity module
  - Positional parameters weren't needed anymore as it wasn't necessary for testing.

### 4/8/2025
- Implemented drone commands script along with parameter conversion successfully
- Found more issues with implementation within the Julia side, started debugging

### 4/10/2025
- Updated Install Script and README to assist users with installing the system
- Finalized drone commands script, and the drones are fully functioning!
<img width="953" alt="Screenshot 2025-04-22 at 2 56 11 PM" src="https://github.com/user-attachments/assets/fb600cb9-e335-4b9a-bf65-5cfce6d8cf43" />

### 4/15/2025
- Started object avoidance implementation, had to do a lot of research still
- Debugged ROS2 dependency as its important to LiDAR

### 4/17/2025
- Worked on object avoidance, still not working properly
- Updated Install Script and README
- Helped finalize the Poster

### 4/22/2025
- Still working on object avoidance, having difficulties
- Finalized the Install Script and README, so it is easier to install and implement on any UBUNTU 22.04 machine
- Finalized poster for real this time

### 4/25/2025




  










