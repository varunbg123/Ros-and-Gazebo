#!/usr/bin/env python3

"""
This script reads a ROS bag file and prints the messages from a specific topic.
"""

import rospy
import rosbag


def read_bag_file(bag_file_path, topic_name):
    """
    Reads a ROS bag file and prints the messages from the specified topic.

    Args:
        bag_file_path (str): Path to the ROS bag file.
        topic_name (str): Name of the topic to read messages from.
    """
    # Initialize the ROS node
    rospy.init_node("read_bag")

    # Open the ROS bag file
    bag = rosbag.Bag(bag_file_path)

    # Read the messages from the specified topic
    # Iterate over the messages in the bag file for the specified topic
    for topic, msg, t in bag.read_messages(topics=["/sam", "/jenny"]):
        # Print the message and its timestamp
        # Print the Sam's data
        if topic == "/sam":
            print("Sam's Data is:", end=" ")
            print(msg.data)
            # Print the time in seconds
            print("Time is:", end=" ")
            print(str(t.to_sec()) + " secs")

        if topic == "/jenny":
            print("Jenny's Data is:", end=" ")
            print(msg.data)
            # Print the time in seconds
            print("Time is:", end=" ")
            print(str(t.to_sec()) + " secs")


# Example usage
read_bag_file("../bag/newtest.bag", "/sam")
