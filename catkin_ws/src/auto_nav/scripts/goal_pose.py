#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Define the callbacks for the action client
def active_cb(extra):
    """
    Callback for when the goal pose is being processed

    :param extra: extra information
    """
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    """
    Callback for when the robot is moving to the goal pose

    :param feedback: feedback from the action server
    """
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    """
    Callback for when the goal pose is reached

    :param status: status of the action (3: success, 2/8: cancelled, 4: aborted)
    :param result: result of the action
    """
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

# Initialize the node
rospy.init_node('goal_pose')

# Create an action client for the move_base action
navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

# Define the goal pose
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

# Set the goal pose
goal.target_pose.pose.position.x = -2.16
goal.target_pose.pose.position.y = 0.764
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.662
goal.target_pose.pose.orientation.w = 0.750

# Send the goal pose to the action server
navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
    rospy.logerr("Action server not available!")
else:
    rospy.loginfo ( navclient.get_result())

