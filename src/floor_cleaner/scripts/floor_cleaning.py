#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
import geometry_msgs.msg

class FloorCleaner:
    def __init__(self):
        rospy.init_node("floor_cleaner")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        # Get the map
        rospy.loginfo("Waiting for the map...")
        self.map = rospy.wait_for_message("/map", OccupancyGrid)
        rospy.loginfo("Map received!")

        # Process map data
        self.map_data = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        self.resolution = self.map.info.resolution
        self.origin = (self.map.info.origin.position.x, self.map.info.origin.position.y)

        # Generate waypoints
        self.waypoints = self.generate_waypoints()

        # Set up the Marker publisher
        self.marker_pub = rospy.Publisher('robot_trail', Marker, queue_size=10)
        
        # Store robot's previous positions
        self.trail_points = []

    def generate_waypoints(self):
        """Generate waypoints in a boustrophedon (zigzag) pattern."""
        waypoints = []
        height, width = self.map_data.shape
        grid_resolution = int(1.0 / self.resolution)  # Adjust based on cleaning width (1 meter here)

        for row in range(0, height, grid_resolution):
            for col in range(0, width, grid_resolution):
                if self.map_data[row, col] == 0:  # Check for free space
                    x = self.origin[0] + col * self.resolution
                    y = self.origin[1] + row * self.resolution
                    waypoints.append((x, y))
            waypoints.reverse() if row % 2 else None  # Zigzag pattern

        rospy.loginfo(f"Generated {len(waypoints)} waypoints for coverage.")
        return waypoints

    def send_goal(self, x, y):
        """Send a goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Simplified orientation
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Reached waypoint ({x}, {y})")
            return True
        else:
            rospy.logwarn(f"Failed to reach waypoint ({x}, {y})")
            return False

    def update_trail(self, x, y):
        """Update and publish the trail of the robot."""
        self.trail_points.append((x, y))

        # Publish trail as markers
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Width of the trail line
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Add each position as a point in the trail
        for point in self.trail_points:
            p = geometry_msgs.msg.Point()
            p.x, p.y = point
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def clean(self):
        """Navigate through all waypoints and leave a trail."""
        for x, y in self.waypoints:
            if not rospy.is_shutdown():
                self.send_goal(x, y)
                self.update_trail(x, y)

if __name__ == "__main__":
    try:
        cleaner = FloorCleaner()
        cleaner.clean()
        rospy.loginfo("Cleaning complete!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Cleaning interrupted.")
