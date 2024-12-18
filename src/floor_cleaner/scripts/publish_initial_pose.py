#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # Set initial pose values
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"

    initial_pose.pose.pose.position.x = -9.0  # Adjust as needed
    initial_pose.pose.pose.position.y = -9.0  # Adjust as needed
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    initial_pose.pose.covariance = [0.1] * 36  # Small covariance for better accuracy

    # Publish the pose
    rospy.sleep(2)  # Give time for publisher to register
    pub.publish(initial_pose)
    rospy.loginfo("Initial pose published!")

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
