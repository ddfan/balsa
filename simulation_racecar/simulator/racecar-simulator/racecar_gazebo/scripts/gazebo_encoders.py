#!/usr/bin/env python

'''
Found from: https://github.com/mindThomas/JetsonCar-Simulation/blob/master/jetsoncar_gazebo/scripts/encoder.py
Script to parse /jetsoncar/joint_states topic and publish encoder values to /encoder/front and /encoder/rear
'''


import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import numpy as np
import math

class EncoderNode:
    def __init__(self):
        # Subscribe to joint_states
        rospy.Subscriber('/racecar/joint_states', JointState, self.joint_states_update)

        # Set publishers
        self.pub_front_left = rospy.Publisher('/encoder/front_left', Int32, queue_size=1)
        self.pub_front_right = rospy.Publisher('/encoder/front_right', Int32, queue_size=1)
        self.pub_rear_left = rospy.Publisher('/encoder/rear_left', Int32, queue_size=1)
        self.pub_rear_right = rospy.Publisher('/encoder/rear_right', Int32, queue_size=1)
        self.pub_timestamp = rospy.Publisher('/encoder/timestamp', Int32, queue_size=1)

    def joint_states_update(self, msg):
        recieved_timestamp = rospy.Time.now()
	# Find the index of the wheels
        try:
            idxFrontLeft = msg.name.index('left_front_wheel_joint')
            idxFrontRight = msg.name.index('right_front_wheel_joint')
            idxRearLeft = msg.name.index('left_rear_wheel_joint')
            idxRearRight = msg.name.index('right_rear_wheel_joint')

        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract encoder angles in radians
            encFrontLeft = msg.position[idxFrontLeft]
            encFrontRight = msg.position[idxFrontLeft]
            encRearLeft = msg.position[idxRearLeft]
            encRearRight = msg.position[idxRearRight]

            # Convert radians into ticks using a gearing ratio of 40 and 12 ticks pr. rev
            encFrontLeft = (encFrontLeft / (2*math.pi)) * (40*12)
            encFrontRight = (encFrontRight / (2*math.pi)) * (40*12)
            encRearLeft = (encRearLeft / (2*math.pi)) * (40*12)
            encRearRight = (encRearRight / (2*math.pi)) * (40*12)

            # Prepare the data for publishing
            encFrontLeftmsg = Int32()
            encFrontLeftmsg.data = encFrontLeft
            encFrontRightmsg = Int32()
            encFrontRightmsg.data = encFrontRight
            encRearLeftmsg = Int32()
            encRearLeftmsg.data = encRearLeft
            encRearRightmsg = Int32()
            encRearRightmsg.data = encRearRight
            # print('encFront', encFront)
            timestamp = Int32()
            timestamp.data = (recieved_timestamp.secs * 1000) + (recieved_timestamp.nsecs / 1000000)  # publish milliseconds

            self.pub_front_left.publish(encFrontLeftmsg)
            self.pub_front_right.publish(encFrontRightmsg)
            self.pub_rear_left.publish(encRearLeftmsg)
            self.pub_rear_right.publish(encRearRightmsg)
            self.pub_timestamp.publish(timestamp)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_encoders_node")
    node = EncoderNode()
    r = rospy.Rate(1)
    while not rospy.core.is_shutdown():
        r.sleep()
