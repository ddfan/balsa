#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf.transformations as tr

from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from controller_adaptiveclbf.cfg import figureEightConfig as ConfigType

from nav_msgs.msg import Odometry

class FigureEightNode(object):
    def __init__(self):        
        """Read in parameters."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = np.pi/4

        self.start_time = rospy.get_rostime()
        self.prev_time = rospy.Time(0)
        self.enable = True

        self.params={}
        self.params["width"] = rospy.get_param('~width',1.0)
        self.params["height"] = rospy.get_param('~height',1.0)
        self.params["speed"] = rospy.get_param('~speed',1.0)
        self.params["freq"] = rospy.get_param('~freq',0.1)
        self.params["x_offset"] = rospy.get_param('~x_offset',0.0)
        self.params["y_offset"] = rospy.get_param('~y_offset',0.0)

        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)

        # Create publishers
        self.pub_target = rospy.Publisher('target', Odometry, queue_size = 1)

        # Initialize message variables.
        self.enable = rospy.get_param('~enable', True)

    def timer_cb(self):
        if not self.enable:
            return

        curr_time = rospy.Time.now()

        t = (curr_time - self.start_time).to_sec()

        # for figure eight
        arclength = self.params["width"] * 6
        omega = t * 2 * np.pi * self.params["speed"] / arclength
        # x_new = self.params["height"] * np.sin(omega)
        y_new = self.params["width"] * np.sin(omega) #* np.cos(omega)
        x_new = self.params["height"] * np.cos(omega)

        # # for sinusoid
        # y_new = self.params["speed"] * t
        # x_new = self.params["width"] * np.sin(2 * np.pi * t * self.params["freq"])

        # Lemniscate of bernoulli (fig 8)
        # x_new = self.params["width"] * np.sqrt(2.0) * np.cos(omega) / (np.sin(omega)**2+1)
        # y_new = self.params["width"] * np.sqrt(2.0) * np.cos(omega) * np.sin(omega) / (np.sin(omega)**2+1)
        
        yaw_new = np.arctan2(y_new - self.y, x_new - self.x)
        if self.prev_time == rospy.Time(0):
            self.vel = 0
        else:
            self.vel = np.sqrt((self.x - x_new)**2 + (self.y - y_new)**2) / ((curr_time - self.prev_time).to_sec() + 1e-6)
        self.x = x_new
        self.y = y_new
        self.yaw = yaw_new
        self.prev_time = curr_time

        # publish reference pose for visualization
        target_msg = Odometry()
        target_msg.header.frame_id = rospy.get_namespace() + "odom"
        target_msg.header.stamp = rospy.get_rostime()
        target_msg.pose.pose.position.x = self.x + self.params["x_offset"]
        target_msg.pose.pose.position.y = self.y + self.params["y_offset"]
        target_msg.pose.pose.position.z = 0.0
        target_msg.pose.pose.orientation.x = 0.0
        target_msg.pose.pose.orientation.y = 0.0
        target_msg.pose.pose.orientation.z = np.sin(self.yaw/2.0)
        target_msg.pose.pose.orientation.w = np.cos(self.yaw/2.0)
        target_msg.child_frame_id = 'base_link'
        target_msg.twist.twist.linear.x = self.vel

        self.pub_target.publish(target_msg)

    def reconfigure_cb(self, config, dummy):
        """Create a callback function for the dynamic reconfigure server."""
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.params["width"] = config["width"]
        self.params["height"] = config["height"]
        self.params["speed"] = config["speed"]
        self.params["x_offset"] = config["x_offset"]
        self.params["y_offset"] = config["y_offset"]
        self.params["freq"] = config["freq"]

        # Check to see if node should be started or stopped.
        if self.enable != config["enable"]:
            if config["enable"]:
                self.start_time = rospy.get_rostime()
        self.enable = config["enable"]

        # Return the new variables.
        return config

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('figure_eight')
    node = FigureEightNode()
    rate = rospy.get_param('~rate', 50.0)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        node.timer_cb()
        r.sleep()
