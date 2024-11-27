# This code is based on the Intelligent Quad Tutorial Repository
# Repository URL: https://github.com/Intelligent-Quads
# License: MIT 

#! /usr/bin/env python
# Import ROS.
import rospy
# Import LaserScan message from package sensor_msgs.
from sensor_msgs.msg import LaserScan
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
# Import the needed math functions.
from math import cos, sin, radians


# Create API objects for both drones
drone1 = gnc_api()  # This drone has the lidar sensor
drone2 = gnc_api()  # This drone does not have the lidar sensor


def laser_cb(msg):
    # Callback function to handle lidar data from drone1
    cr_scan = msg
    avoid_x = 0.0
    avoid = False

    # Check for obstacles in front of drone1 (within 4 units)
    for i in range(1, len(cr_scan.ranges)):
        if cr_scan.ranges[i] < 4.0 and cr_scan.ranges[i] > 0.35:
            avoid = True
            break

    if avoid:
        # Get the current heading of drone1
        cr_heading = radians(drone1.get_current_heading())
        # Move right by 3 units to avoid collision
        avoid_x = 3.0 * cos(cr_heading + radians(90))  # Moving right

        cur_pose = drone1.get_current_location()
        # Sending the new goal to move right and continue forward
        drone1.set_destination(cur_pose.x + avoid_x, cur_pose.y, 3, 0)


def main():
    # Initialize the ROS node
    rospy.init_node("multi_drone_controller", anonymous=True)

    # Drone 1 (with lidar) setup
    drone1.wait4connect()
    drone1.set_mode("GUIDED")
    drone1.initialize_local_frame()
    drone1.takeoff(3)

    # Drone 2 setup
    drone2.wait4connect()
    drone2.set_mode("GUIDED")
    drone2.initialize_local_frame()
    drone2.takeoff(3)

    # Start the lidar subscriber for drone1
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)

    # Set the rate of control loop
    rate = rospy.Rate(3)

    # Waypoints for both drones (start 10 units apart and move towards each other)
    drone1_goals = [[0, 0, 3, 0], [10, 0, 3, 0]]  # Drone 1 moves forward
    drone2_goals = [[10, 0, 3, 0], [0, 0, 3, 0]]  # Drone 2 moves towards drone1
    i = 0

    while i < len(drone1_goals):
        # Set waypoints for both drones
        drone1.set_destination(drone1_goals[i][0], drone1_goals[i][1], drone1_goals[i][2], drone1_goals[i][3])
        drone2.set_destination(drone2_goals[i][0], drone2_goals[i][1], drone2_goals[i][2], drone2_goals[i][3])
        
        rate.sleep()
        
        # Check if both drones have reached their current waypoints
        if drone1.check_waypoint_reached() and drone2.check_waypoint_reached():
            i += 1

    # Land both drones after waypoints are reached
    drone1.land()
    drone2.land()

    rospy.loginfo(CGREEN2 + "All waypoints reached. Both drones landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
