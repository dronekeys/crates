#!/usr/bin/env python

# Import required ROS libraries
import roslib

# Import these CRATES packages
roslib.load_manifest('hal_quadrotor')
roslib.load_manifest('sim')

# Use the ROS python interface
import rospy

# Import custom messages and topics from the simulator
from sim.srv import *

# Import custom messages and topics from the quadrotor HAL
from hal_quadrotor.msg import *

# Create a callback function for the subscriber.
def callback(data):

    # Simply print out values in our custom message.
    rospy.loginfo("Quadrotor position: [%f,%f,%f]", data.x, data.y, data.z)

# This ends up being the main while loop.
def listener():

    # Wait for the Pause service to appear, then Pause the simulator
    rospy.wait_for_service('/simulator/Pause');
    try:
        service  = rospy.ServiceProxy('/simulator/Pause', Pause)
        response = service()
    except rospy.ServiceException, e:
        print "Service call failed: %s"

    # Wait for the Pause service to appear, then Pause the simulator
    rospy.wait_for_service('/simulator/Resume');
    try:
        service  = rospy.ServiceProxy('/simulator/Resume', Resume)
        response = service()
    except rospy.ServiceException, e:
        print "Service call failed: %s"

    # Wait for the Pause service to appear, then Pause the simulator
    rospy.wait_for_service('/simulator/Insert');
    try:
        service  = rospy.ServiceProxy('/simulator/Insert', Insert)
        response = service("UAV0","model://hummingbird")
    except rospy.ServiceException, e:
        print "Service call failed: %s"

    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber('/hal/UAV0/Estimate', State, callback)

    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':

    # Initialize the node and name it.
    rospy.init_node('example', anonymous = True)
    
    # Go to the main loop.
    listener()