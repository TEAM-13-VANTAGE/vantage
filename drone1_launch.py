import time
from pymavlink import mavutil

# Initialize the connection to the drone
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')  # Replace with your connection string
master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
print("Heartbeat received from Drone 1")

# Define the GUIDED mode
GUIDED_MODE = 'GUIDED'

# Set the mode to GUIDED
master.set_mode(GUIDED_MODE)
time.sleep(2)

# Arm the drone
master.arducopter_arm()
master.motors_armed_wait()
print("Drone 1 Armed")

# Take off to 10 meters
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 10
)
print("Drone 1 Takeoff to 10m")
