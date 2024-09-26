#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

# Callback Function
def callback(msg):
    print(msg)

# Init
rospy.init_node("subscriber")
rospy.Subscriber("topic", Int64, callback=callback)

# Spinning
rospy.spin()