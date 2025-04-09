import subprocess
import time
import os

# Step 1: Run the Julia program "test.jl" located in Particle-Simulation folder
def run_julia_program():
    julia_program = os.path.join(os.getcwd(), 'Particle-Simulation', 'test.jl')
    print("Running Julia simulation...")
    # Wait for the Julia program to finish before proceeding
    subprocess.run(['julia', julia_program], check=True)
    print("Julia simulation completed.")

# Step 3: Launch Gazebo with the ROS launch file "multi_drone.launch"
def launch_gazebo():
    launch_file = os.path.join(os.getcwd(), 'gazebo', 'iq_sim', 'launch', 'multi_drone.launch')
    print("Launching Gazebo with ROS...")
    subprocess.run(['roslaunch', launch_file], check=True)
    print("Gazebo launched.")

# Step 4: Wait 30 seconds and then run the ArduCopter vehicle simulation
def run_arducopter():
    print("Waiting for 30 seconds before launching ArduCopter simulations...")
    time.sleep(30)
    
    vehicle_command1 = os.path.expanduser('sim_vehicle.py')
    vehicle_command2 = os.path.expanduser('sim_vehicle.py')
    
    # Launch the first drone
    subprocess.Popen([vehicle_command1, '-v', 'ArduCopter', '-f', 'gazebo-drone1', '-I0'])
    
    # Launch the second drone
    subprocess.Popen([vehicle_command2, '-v', 'ArduCopter', '-f', 'gazebo-drone1', '-I1'])
    
    print("ArduCopter simulations started for drone1 and drone2.")

# Step 5: Wait 1 minute and display a countdown
def countdown(wait_time):
    print(f"Waiting for {wait_time} seconds with a countdown...")
    for i in range(wait_time, 0, -1):
        print(f"{i} seconds remaining")
        time.sleep(1)
    print("Countdown completed.")

# Step 6: Run the obs_avoid.py file located in the gazebo folder
def run_obs_avoid():
    obs_avoid_script = os.path.join(os.getcwd(), 'gazebo', 'drone_lidar_avoidance.py')
    print("Running obstacle avoidance script...")
    subprocess.Popen(['python3', obs_avoid_script], check=True)
    print("Obstacle avoidance script completed.")

# Main function to execute the steps sequentially
def main():
    # Step 1: Run Julia program and wait for it to complete
    run_julia_program()
    # Step 3: Launch Gazebo and wait for it to complete
    # launch_gazebo()
    # # Step 4: Wait 30 seconds and run ArduCopter simulation
    # run_arducopter()
    # # Step 5: Wait 1 minute and display countdown
    countdown(30)
    # Step 6: Run obstacle avoidance script
    run_obs_avoid()

if __name__ == "__main__":
    main()
