#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

# Node + Topic Initialization
rospy.init_node("publisher_node")
pub = rospy.Publisher(name="topic", data_class=Int64, queue_size=1)


# Looping
while not rospy.is_shutdown():
    pub.publish(1)
    rospy.sleep(1)