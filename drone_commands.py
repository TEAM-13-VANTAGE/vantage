import time
from pymavlink import mavutil

def launch_drone(master):
    # Define the GUIDED mode
    GUIDED_MODE = 'GUIDED'

    # Set the mode to GUIDED
    master.set_mode(GUIDED_MODE)
    time.sleep(2)

    # Arm the drone
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone" + id +  "Armed")

def drone_takeoff(master, height):
    # Take off to 10 meters
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 10
)
    print("Drone" + id +  "takeoff to" + height + "meters")
    
    return


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
    time_boot_ms, target_system, target_component, coordinate_frame,
    type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
):

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.SET_POSITION_TARGET_LOCAL_NED,
        time_boot_ms, target_system, target_component, coordinate_frame,
        type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    )
    return
    