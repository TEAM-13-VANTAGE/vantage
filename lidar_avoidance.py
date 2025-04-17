# lidar_avoidance_module.py
from rplidar import RPLidar
import time
from pymavlink import mavutil  # Only needed for set_velocity_body()
import drone_commands

# === CONFIG ===
PORT_NAME = '/dev/ttyUSB0'  # Replace as needed


# === LiDAR Functions ===
def connect_lidar(port=PORT_NAME):
    try:
        print("Connecting to LiDAR...")
        lidar = RPLidar(port)
        print("LiDAR connected.")
        return lidar
    except Exception as e:
        print(f"LiDAR connection failed: {e}")
        return None

def get_obstacle_direction(scan, threshold):
    """Returns 'front', 'left', 'right', or 'none'."""
    sectors = {'front': [], 'left': [], 'right': []}
    for angle, distance in scan:
        if distance == 0:
            continue
        if (345 <= angle or angle <= 15):  # Front
            sectors['front'].append(distance)
        elif 60 <= angle <= 120:  # Left
            sectors['left'].append(distance)
        elif 240 <= angle <= 300:  # Right
            sectors['right'].append(distance)

    for direction, distances in sectors.items():
        if distances and min(distances) < threshold:
            return direction
    return 'none'

def fetch_obstacle_direction(lidar, threshold):
    """Fetch a single scan and return obstacle direction."""
    for scan in lidar.iter_scans(max_buf_meas=500):
        scan_data = [(angle, distance) for _, angle, distance in scan]
        return get_obstacle_direction(scan_data, threshold)

# === Drone yaw Control ===
def set_velocity_body(master, drone_speed, yaw, yaw_rate, duration=1):
    """Sends velocity command in body frame."""
    msg = drone_commands.send_position_target_local_ned(master =master,
        time_boot_ms=0,
        target_system=master.target_system,
        target_component=master.target_component,
        coordinate_frame=1,
        type_mask=0b110111000000,  # Use position only
        x=0, y=0, z=-10,  # Maintain altitude
        vx=drone_speed, vy=0, vz=0,  # Forward speed
        afx=0, afy=0, afz=0,  # No acceleration
        yaw=yaw, yaw_rate=yaw_rate)  # Yaw control
    
    for _ in range(duration * 10):
        master.mav.send(msg)
        time.sleep(0.1)

# === Manual Maneuvering ===
def perform_avoidance(master, mode, params):
    """
    mode options:
        'Horizontal' – sidestep right
        'Vertical' – ascend
        'Right-of-way' – stop & hover
    """
    drone_speed = params['drone_speed']
    yaw = params['yaw']
    yaw_rate = params['yaw_rate']
    
    if mode == 'Horizontal':
        print("Executing horizontal avoidance: ")
        set_velocity_body(master, drone_speed, yaw, yaw_rate, duration=2)
    elif mode == 'Vertical':
        print("Executing vertical avoidance: ascend.")
        set_velocity_body(master, drone_speed, yaw, yaw_rate, duration=2)
    elif mode == 'Row':
        print("Executing right-of-way: hover.")
        set_velocity_body(master, drone_speed, yaw, yaw_rate, duration=2)
    else:
        print("Unknown maneuver mode. No action taken.")
