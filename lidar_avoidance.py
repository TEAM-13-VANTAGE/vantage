import time
import threading
from pymavlink import mavutil
import drone_commands
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# === Lidar Processor Node ===
class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.front = self.left = self.right = float('inf')
        self.threshold = 1.5  # meters
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

    def lidar_callback(self, msg):
        try:
            self.front = msg.ranges[0]
            self.left = msg.ranges[90]
            self.right = msg.ranges[270]
        except IndexError:
            self.get_logger().warn("Scan data too short!")

    def get_obstacle_direction(self):
        if self.front < self.threshold:
            return 'front'
        elif self.left < self.threshold:
            return 'left'
        elif self.right < self.threshold:
            return 'right'
        return 'none'


    def send_position_command(master, x, y, vx):
        drone_commands.send_position_target_local_ned(
            master=master,
            time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            target_system=master.target_system,
            target_component=master.target_component,
            coordinate_frame=1,
            type_mask=0b110111000000,
            x=x, y=y, z=-10,
            vx=vx
        )

    def perform_avoidance(master, direction):
        if direction == 'front' or direction == 'left':
            yaw_rate = -0.5  # Turn right
        elif direction == 'right':
            yaw_rate = 0.5   # Turn left
        else:
            return

        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b110111111000,  # Only angular.z enabled
            0, 0, 0,         # Position
            0, 0, 0,         # Velocity
            0, 0, yaw_rate   # Acceleration & yaw rate
        )