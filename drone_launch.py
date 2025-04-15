import time
from pymavlink import mavutil
import drone_commands

# Initialize the connection to the drone
def init_drone_0(x, y, z, drone_speed):
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
    type_mask=0b110111111000,
    x=x, y=y, z=z,
    vx=drone_speed, vy=0, vz=0,
    afx=0, afy=0, afz=0,
    yaw=0, yaw_rate=0
)
    
# Initialize the connection to the drone
def init_drone_1(x, y, z, drone_speed):
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
    type_mask=0b110111111000,
    x=x, y=y, z=z,
    vx=drone_speed, vy=0, vz=0,
    afx=0, afy=0, afz=0,
    yaw=0, yaw_rate=0
)



    