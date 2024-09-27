#!/usr/bin/env python3

"""
Sets the initial pose of the robot in the map.

Uses the initial pose from Gazebo to set the initial pose of the robot in the
map. This is necessary because the robot's initial pose is not known by the
SLAM algorithm.

The node publishes a PoseWithCovarianceStamped message to the /initialpose topic
with the initial pose of the robot in the map.

The node waits for the /odom topic to be published before setting the initial
pose. This is necessary because the /odom topic is not published until the
robot is spawned in Gazebo.

The node sleeps for 1 second before publishing the initial pose. This is
necessary because the /initialpose topic is not ready to receive messages
until the SLAM algorithm is fully initialized.

The node logs a message when the initial pose is set.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"

# Wait for the odometry topic to be published. This is necessary because the
# SLAM algorithm is not yet initialized when the robot is spawned in Gazebo,
# and the odometry topic is not published until the robot is spawned.
odom_msg = rospy.wait_for_message('/odom', Odometry)

# Get initial pose from Gazebo. The initial pose of the robot is not known
# by the SLAM algorithm, so we need to get it from Gazebo.
init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

# Delay. This is necessary because the /initialpose topic is not ready to
# receive messages until the SLAM algorithm is fully initialized.
rospy.sleep(1)

# Publish message. This will set the initial pose of the robot in the map.
rospy.loginfo("setting initial pose")
pub.publish(init_msg)
rospy.loginfo("initial pose is set")
