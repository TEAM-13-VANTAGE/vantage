import time
from pymavlink import mavutil
import drone_commands
import lidar_avoidance
import csv

def init_drone_0(params, maneuver):
    # Fetch scenario parametersz
    hf_params = drone_commands.get_high_fidelity_params(params, maneuver)
    drone_speed = float(hf_params['drone_speed'])

    # Connect to drone
    port = 14550
    master = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
    print("Waiting for Drone 0...")
    master.wait_heartbeat()
    print("Heartbeat received.")
    time.sleep(10)

    # Launch the drone
    print("Launching Drone...")
    drone_commands.launch_drone(master)
    drone_commands.drone_takeoff(master, 10)
    drone_commands.send_position_target_local_ned(master=master,
    time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
    target_system=master.target_system,
    target_component=master.target_component,
    coordinate_frame=1,  # MAV_FRAME_LOCAL_NED
    type_mask=0b110111000000,  # Position only
    x=0, y=0, z=-10,  # NED => z = -altitude
    vx=drone_speed)

    # Start ROS 2 node in background thread
    lidar_avoidance.rclpy.init()
    lidar_node = lidar_avoidance.LidarProcessor()
    executor_thread = lidar_avoidance.threading.Thread(target=lidar_avoidance.rclpy.spin, args=(lidar_node,), daemon=True)
    executor_thread.start()


    with open("drone0_telemetry.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "x", "y", "z", "speed", "collision_violation", "direction"])
        try:
            while True:
                direction = lidar_node.get_obstacle_direction()
                violation = direction != 'none'

                # Read telemetry
                msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                if msg:
                    x = msg.x
                    y = msg.y
                    z = msg.z
                    vx = msg.vx
                    vy = msg.vy
                    vz = msg.vz
                    speed = (vx**2 + vy**2 + vz**2)**0.5

                    # Print and log
                    print(f"[Telemetry] x: {x:.2f} | y: {y:.2f} | z: {z:.2f} | speed: {speed:.2f} | obstacle: {direction}")
                    writer.writerow([time.time(), x, y, z, speed, violation, direction])

                # Handle avoidance
                if violation:
                    print(f"Avoiding {direction}")
                    lidar_avoidance.perform_avoidance(master, direction)
                else:
                    drone_commands.send_position_target_local_ned(
                        master=master,
                        time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                        target_system=master.target_system,
                        target_component=master.target_component,
                        coordinate_frame=1,
                        type_mask=0b100111000111,
                        x=0, y=0, z=-10,
                        vx=drone_speed
                    )

                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down")
            lidar_avoidance.rclpy.shutdown()



# Initialize the connection to the drone
def init_drone_1(params, maneuver):
    print("Initializing Drone 1...")
    hf_params = drone_commands.get_high_fidelity_params(params, maneuver)
    heli_speed = float(hf_params['heli_speed'])
    
    port = 14560
    master = mavutil.mavlink_connection('udp:127.0.0.1:' + str(port))  # Replace with your connection string
    
    print("Waiting for Drone 1...")
    master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
    print("Heartbeat received from Drone 1")
    time.sleep(10)
    drone_commands.launch_drone(master)
    drone_commands.drone_takeoff(master, 10)
    drone_commands.send_position_target_local_ned(
    master=master,
    time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
    target_system=master.target_system,
    target_component=master.target_component,
    coordinate_frame=1,  # MAV_FRAME_LOCAL_NED
    type_mask=0b110111000000,  # Position only
    x=0, y=0, z=-10,  # NED => z = -altitude
    vx=heli_speed
)
    time.sleep(1)
    

    