import time
from pymavlink import mavutil
import drone_commands
import lidar_avoidance
    
# Initialize the connection to the drone
def init_drone_0(params, maneuver):
    hf_params = drone_commands.get_high_fidelity_params(params, maneuver)
    x_pos = float(hf_params['drone_x_pos'])
    y_pos = float(hf_params['drone_y_pos'])
    drone_speed = float(hf_params['drone_speed'])
    
    port = 14550
    master = mavutil.mavlink_connection('udp:127.0.0.1:' + str(port))  # Replace with your connection string
    print("Waiting for Drone 0...")
    master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
    print("Heartbeat received from Drone 0")
    time.sleep(10)
    print("Launching Drone...")
    drone_commands.launch_drone(master)
    # drone_commands.drone_takeoff(master, 10)
    drone_commands.send_position_target_local_ned(master,
    time_boot_ms=0,
    target_system=master.target_system,
    target_component=master.target_component,
    coordinate_frame=1,
    type_mask=0b110111000000,
    x=x_pos, y=y_pos, z=-10,
    vx=drone_speed
)
    lidar=lidar_avoidance.connect_lidar()
    if lidar:
        try:
            while True:
                scan = lidar.fetch_obstacle_direction(lidar, maneuver)
                print(f"Obstacle direction: {scan}")
                if scan != 'none':
                    if maneuver != '':
                        lidar_avoidance.perform_avoidance(master, scan)
                else:
                    drone_commands.send_position_target_local_ned(master,
                    time_boot_ms=0,
                    target_system=master.target_system,
                    target_component=master.target_component,
                    coordinate_frame=1,
                    type_mask=0b110111000000,
                    x=x_pos, y=y_pos, z=-10,
                    vx=drone_speed
                    )
        except KeyboardInterrupt:
            print("Shutting down.")
            lidar.stop()
            lidar.disconnect()

# Initialize the connection to the drone
def init_drone_1(params, maneuver):
    print("Initializing Drone 1...")
    hf_params = drone_commands.get_high_fidelity_params(params, maneuver)
    x_pos = float(hf_params['drone_x_pos'])
    y_pos = float(hf_params['drone_y_pos'])
    heli_speed = float(hf_params['heli_speed'])
    
    port = 14560
    master = mavutil.mavlink_connection('udp:127.0.0.1:' + str(port))  # Replace with your connection string
    
    print("Waiting for Drone 1...")
    master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
    print("Heartbeat received from Drone 1")
    time.sleep(10)
    drone_commands.launch_drone(master)
    # drone_commands.drone_takeoff(master, 10)
    drone_commands.send_position_target_local_ned(master,
    time_boot_ms=0,
    target_system=master.target_system,
    target_component=master.target_component,
    coordinate_frame=1,
    type_mask=0b110111000000,
    x=x_pos, y=y_pos, z=-10,
    vx=heli_speed
    )
    