As of now, these files are compatible only with Ubuntu 22.04, ROS2 Humble, and Gazebo Harmonic. They have not been tested with other versions of the software.

# Installation and Setup
1. Install Ubuntu 22.04 VM and complete the setup. _(Preferable specs: **50GB storage**, **8GB memory**)_
2. Copy the `install_script.sh` to the `~/` directory using the command:
    ```bash
    cp -r ~/vantage/ubuntu_files/install_script.sh ~/
    ```
3. Run the following commands in a terminal:
    ```bash
    chmod +x ~/install_script.sh
    ~/install_script.sh
    ```
4. Open three terminals:

     - **Terminal 1**: Run
        ```bash
        gz sim -v4 -r iris_runway.sdf
        ```
     - **Terminal 2**: Run
        ```bash
        sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --model JSON
        ```
     - **Terminal 3**: Run
        ```bash
        sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --console --model JSON
        ```
5. The installation is successful if the system opens these three tabs and the console does not say `link 1 down`. _If this is not the case, redo step 3._
    <img width="800" alt="image" src="https://github.com/user-attachments/assets/299e9784-3558-4eb3-b06b-d6e7f5297d0e">

6. There should be two drones shown in Gazebo, and both should be controllable by their respective consoles, as shown below:
     <img width="800" alt="image" src="https://github.com/user-attachments/assets/b79dde47-4d78-4ec4-ab86-01c437419507">

7. Press `Ctrl+C` on all terminals and close them.

8. Run the following command to start the system:
     ```bash
     python3 gui.py
     ```