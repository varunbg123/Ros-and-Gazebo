#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse


def callback(req):
    """
    This function is the callback for the ROS service.
    It takes a request Message from the client and returns a response Message.

    Args:
        req (SetBoolRequest): The request message received from the client.

    Returns:
        SetBoolResponse: The response message to be sent back to the client.
    """
    # Create the response Message
    response = SetBoolResponse()

    # If the request data is True, enable the device
    if req.data == True:
        response.success = True
        response.message = "The device was enabled"
    
    # If the request data is False, disable the device
    else:
        response.success = True
        response.message = "The device was disabled"
    
    # Return the response Message
    return response

# Node setup
# Initialize the ROS node
rospy.init_node("server")

# Create a ROS service with the name "test_service"
# and the callback function "callback"
# This service will handle requests of type SetBool and will call the callback function
# whenever a request is received.
rospy.Service("test_service", SetBool, callback)

# Keep the node alive
# This will keep the node running until it is shut down
# The rospy.spin() function puts the node in a loop and keeps it alive.
# It will continue to process messages until the node is shut down.
rospy.spin()
