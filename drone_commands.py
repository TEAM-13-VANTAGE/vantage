import time
from pymavlink import mavutil

def get_high_fidelity_params(params, maneuver):
    print("Getting high fidelity parameters...")

    keys = []
    if maneuver == 'Vertical':
        keys = [
            "step", "contactLevel", "scenario", "drone_ascent_rate",
            "drone_direction", "drone_response_distance", "drone_speed",
            "drone_x_pos", "drone_y_pos", "heli_speed", "min_dist"
        ]
    elif maneuver == 'Horizontal':
        keys = [
            "step", "contactLevel", "scenario", "drone_direction", "drone_horizontal_turn_angle", "drone_horizontal_turn_rate", 
            "drone_response_distance", "drone_speed", "drone_x_pos", "drone_y_pos", "heli_speed", "min_dist"
        ]
    elif maneuver == 'Row':
        keys = [
            "step", "contactLevel", "scenario", "drone_direction", "drone_horizontal_turn_angle", "drone_horizontal_turn_rate",
            "drone_response_distance", "drone_speed", "drone_x_pos", "drone_y_pos", "force_right_turn", "heli_speed", "min_dist"
        ]
    else:
        raise ValueError("Invalid maneuver type. Must be 'Vertical', 'Horizontal', or 'Row'.")

    if len(params) < len(keys):
        raise IndexError(
            f"Not enough parameters. Expected {len(keys)} but got {len(params)}.\n"
            f"Maneuver: {maneuver}\n"
            f"Params: {params}"
        )

    hf_params = {key: params[i] for i, key in enumerate(keys)}
    print("High fidelity parameters obtained!")
    return hf_params




def launch_drone(master):
    # Define the GUIDED mode
    print("Setting drone to GUIDED mode...")
    GUIDED_MODE = 'GUIDED'

    # Set the mode to GUIDED
    master.set_mode(GUIDED_MODE)
    
    time.sleep(2)
    
    # Wait for the drone to be arm
    arm_until_success(master)
    print("Drone Armed")

    time.sleep(2)

def arm_until_success(master, timeout=300, retry_interval=15):
    print("Attempting to arm drone...")

    start_time = time.time()
    while time.time() - start_time < timeout:
        # Send arming command
        master.arducopter_arm()
        print("Sent arming command.")

        # Continuously check for arming success within retry_interval
        check_start = time.time()
        while time.time() - check_start < retry_interval:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Motors are armed.")
                return True

    print("Failed to arm within timeout.")
    return False

def update_runway_pose_in_sdf(file_path, new_x, new_y):
    with open(file_path, "r") as file:
        lines = file.readlines()

    for i in range(len(lines)):
        if "<uri>model://runway</uri>" in lines[i]:
            # Search within a few lines after this to find the <pose> tag
            for j in range(i, i + 5):
                if "<pose" in lines[j]:
                    parts = lines[j].split(">")
                    pose_data = parts[1].split("<")[0].split()
                    pose_data[0] = str(new_x)
                    pose_data[1] = str(new_y)
                    new_pose = " ".join(pose_data)
                    lines[j] = f'      <pose degrees="true">{new_pose}</pose>\n'
                    break
            break

    with open(file_path, "w") as file:
        file.writelines(lines)

    print(f"Updated runway pose to x={new_x}, y={new_y}")

def update_drone_pose_in_sdf(file_path, new_x, new_y, drone_name):
    with open(file_path, "r") as file:
        lines = file.readlines()

    for i in range(len(lines)):
        if f"<name>{drone_name}</name>" in lines[i]:
            # Search within a few lines after this to find the <pose> tag
            for j in range(i, i + 5):
                if "<pose" in lines[j]:
                    parts = lines[j].split(">")
                    pose_data = parts[1].split("<")[0].split()
                    pose_data[0] = str(new_x)
                    pose_data[1] = str(new_y)
                    new_pose = " ".join(pose_data)
                    lines[j] = f'      <pose degrees="true">{new_pose}</pose>\n'
                    break
            break

    with open(file_path, "w") as file:
        file.writelines(lines)

    print(f"Updated pose of {drone_name} to x={new_x}, y={new_y}")


def drone_takeoff(master, height, timeout=30):
    # Take off to 10 meters
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, height
)
    print("Drone takeoff to " + str(height) + " meters")
    time.sleep(5)  # Wait for the drone to take off
    
    # Wait until the drone reaches the desired altitude
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            alt = msg.relative_alt / 1000.0  # Convert mm to meters
            print(f"Current altitude: {alt:.2f} m")
            if alt >= height - 0.5:
                print("Target altitude reached.")
                return True
    print("Altitude not reached within timeout.")
    return False


#Parameters:
# 1. time_boot_ms : sender's time since boot in milliseconds
# 2. target_system : system ID of vehicle
# 3. target_component : component ID of vehicle or just 0
# 4. coordinate_frame : 
    # 1 : MAV_FRAME_LOCAL_NED
    # 7 : MAV_FRAME_LOCAL_OFFSET_NED
    # 8 : MAV_FRAME_BODY_NED
    # 9 : MAV_FRAME_BODY_OFFSET_NED    
# 5. type_mask : bitmask to indicate which dimensions should be ignored by the vehicle
    # Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
    # Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
    # Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
    # Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
    # Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
    # Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
    # Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)
# 6. x: X position in meters (positive is forward or North)
# 7. y: Y position in meters (positive is right or East)
# 8. z: Z position in meters (positive is down)
# 9. vx: X velocity in m/s (positive is forward or North)
# 10. vy: Y velocity in m/s (positive is right or East)
# 11. vz: Z velocity in m/s (positive is down)
# 12. afx: X acceleration or force (positive is forward or North)
# 13. afy: Y acceleration or force (positive is right or East)
# 14. afz: Z acceleration or force (positive is down)
# 15. yaw: yaw in radians (counterclockwise, 0 is North)
# 16. yaw_rate: yaw rate in rad/s (positive is counterclockwise)


def send_position_target_local_ned(master,
    time_boot_ms, target_system, target_component, coordinate_frame=1,
    type_mask=0b110111000000, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
):
    print("sending position target local NED...")
    master.mav.set_position_target_local_ned_send(
        time_boot_ms, target_system, target_component, coordinate_frame,
        type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    )
    print("Position target local NED sent.")
    
    