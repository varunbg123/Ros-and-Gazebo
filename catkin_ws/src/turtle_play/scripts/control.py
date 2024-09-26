#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# In order to run the turtlebot from the ros page: http://wiki.ros.org/turtlesim
# rosrun turtlesim turtlesim_node

def callback(msg):
    print(msg)

rospy.init_node("control")

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

rospy.Subscriber("/turtle1/pose", Pose, callback=callback)

msg = Twist()
msg.linear.x = -1
msg.angular.z= 1

for i in range(5):
    pub.publish(msg)
    rospy.sleep(1)

rospy.spin()