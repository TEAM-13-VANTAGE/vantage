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



    def perform_avoidance(master, direction, yaw_rate=0.5):
        
        if direction == 'front' or direction == 'left':
            yaw = yaw_rate  # Turn right
            print (f"Performing avoidance. Turning right with yaw rate: {yaw}")
        elif direction == 'right':
            yaw = yaw_rate  # Turn left
            print (f"Performing avoidance. Turning left with yaw rate: {yaw}")
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
            0, 0, yaw   # Acceleration & yaw rate
        )