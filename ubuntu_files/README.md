As of now These files are compatible only with Ubuntu 22.04, ROS2 Humble, & Gazebo Harmonic. It has not been tested with other versions of the software.

# Installation and Setup
1. Install Ubuntu 22.04 VM and complete set up. _(Preferable specs: **50GB storage**, **8GB memory**)_
2. Install the install_script.sh to the `~/` directory. 
3. Run the following in a terminal:
   `chmod +x ~/install_script.sh` `~/install_script.sh`
4. Once completed, open two new terminals.
5. Run `gz sim -v4 -r iris_runway.sdf` in one terminal.
6. Run `sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --model JSON` on the other.
7. The installation is successful if the system opens these three tabs and the console does not say `link 1 down`. _If this is not the case, redo step 3._
<img width="800" alt="image" src="https://github.com/user-attachments/assets/299e9784-3558-4eb3-b06b-d6e7f5297d0e">

8. Ctrl+C on all terminals and close all.
9. Copy and paste the provided `iris_runway.sdf` file to the directory below. Replace if existing.

`~/ardu_ws/install/ardupilot_gz_gazebo/share/ardupilot_gz_gazebo/worlds`

10. Copy and paste the provided `iris_with_lidar` & `iris_with_lidar_2` folders to the directory below. Replace if existing.

`~/ardu_ws/install/ardupilot_gz_description/share/ardupilot_gz_description/`

11. Open three terminals:

    Terminal 1: Run `gz sim -v4 -r iris_runway.sdf`

    Terminal 2: Run `sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --model JSON`

    Terminal 3: Run `sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --console --model JSON`

12. There should be two drones shown on Gazebo, and both are controllable by their respective consoles  as shown below:
<img width="800" alt="image" src="[https://github.com/user-attachments/assets/b79dde47-4d78-4ec4-ab86-01c437419507]">


