#!/usr/bin/env python3

"""
This script is a client for a ROS service that enables or disables a device.
It initializes a ROS node, creates a service proxy, waits for the service to
become available, sends a request to enable the device, and prints the response.
"""

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

# Initialize the ROS node
rospy.init_node("client")

# Create a service proxy for the "test_service" service
# Service proxy is a convenient way to call a service without manually handling
# the service interface
client = rospy.ServiceProxy("test_service", SetBool)

# Wait for the service to become available
# This is necessary because the service may not be available when the script
# starts running
client.wait_for_service()

# Create a request message
# The request message contains the data to be sent to the service
request = SetBoolRequest()
request.data = True

# Send the request to the service and get the response
response = client(request)

# Print the response
print(response)
