import time
from pymavlink import mavutil
import drone_commands
import lidar_avoidance

def get_high_fidelity_params(self, params, maneuver):

    if maneuver == 'Vertical':
        keys = [
            "step", "contactLevel", "scenario", "drone_ascent_rate",
            "drone_direction", "drone_response_distance", "drone_speed",
            "drone_x_pos", "drone_y_pos", "heli_speed", "min_dist"
        ]
        hf_params = {key: params[i] for i, key in enumerate(keys)}
    if maneuver == 'Horizontal':
        keys = [
            "step", "contactLevel", "scenario", "drone_direction", "drone_horizontal_turn_angle", "drone_horizontal_turn_rate", 
            "drone_response_distance", "drone_speed", "drone_x_pos", "drone_y_pos", "heli_speed", "min_dist"
        ]
        hf_params = {key: params[i] for i, key in enumerate(keys)}
        
    if maneuver == 'Row':
        keys = [
            "step", "contactLevel", "scenario", "drone_direction", "drone_horizontal_turn_angle", "drone_horizontal_turn_rate",
            "drone_response_distance", "drone_speed", "drone_x_pos", "drone_y_pos", "force_right_turn", "heli_speed", "min_dist"
        ]
        hf_params = {key: params[i] for i, key in enumerate(keys)}
    return hf_params
        

    
# Initialize the connection to the drone
def init_drone_0(params, maneuver):
    hf_params = get_high_fidelity_params(params, maneuver)
    x_pos = hf_params['drone_x_pos']
    y_pos = hf_params['drone_y_pos']
    drone_speed = hf_params['drone_speed']
    
    port = 14550
    master = mavutil.mavlink_connection('udp:127.0.0.1:' + port)  # Replace with your connection string

    master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
    print("Heartbeat received from Drone " + id)
    time.sleep(10)
    drone_commands.launch_drone(master)
    drone_commands.drone_takeoff(master, 10)
    drone_commands.send_position_target_local_ned(
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
                    drone_commands.send_position_target_local_ned(
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
    hf_params = get_high_fidelity_params(params, maneuver)
    x_pos = hf_params['drone_x_pos']
    y_pos = hf_params['drone_y_pos']
    heli_speed = hf_params['heli_speed']
    
    port = 14560
    master = mavutil.mavlink_connection('udp:127.0.0.1:' + port)  # Replace with your connection string

    master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
    print("Heartbeat received from Drone " + id)
    time.sleep(10)
    drone_commands.launch_drone(master)
    drone_commands.drone_takeoff(master, 10)
    drone_commands.send_position_target_local_ned(
    time_boot_ms=0,
    target_system=master.target_system,
    target_component=master.target_component,
    coordinate_frame=1,
    type_mask=0b110111000000,
    x=x_pos, y=y_pos, z=-10,
    vx=heli_speed
    )


    