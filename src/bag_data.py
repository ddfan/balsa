#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosbag

from dynamic_reconfigure.client import Client as DynamicReconfigureClient

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Time

import os
import time

BASE_PATH = '/home/nereid/learn2adapt_ws/'
BAG_FILENAME = 'test'

class BagDataNode(object):
	def __init__(self):
		self.bag = rosbag.Bag(BASE_PATH + BAG_FILENAME + '.bag', 'w')

		# dynamic reconfigure client
		self.dyn_reconfig_client = DynamicReconfigureClient('controller_adaptiveclbf_reconfig', timeout=30, config_callback=self.reconfigure_cb)
		# for no learning uncomment below
		# self.dyn_reconfig_client.update_configuration({"model_train": False})

		self.sub_odom = rospy.Subscriber('/downward/vio/odometry', Odometry, self.odom_cb, queue_size=1)
		self.sub_ref = rospy.Subscriber('/reference_vis', Odometry, self.ref_cb, queue_size=1)

		self.odom_msg = Odometry()
		self.ref_msg = Odometry()

		self.start_time = rospy.Time.now()
		self.use_model = False

	def reconfigure_cb(self, config):
		"""Create a callback function for the dynamic reconfigure client."""

	def odom_cb(self, odom_msg):
		if self.start_time == 0:
			self.start_time = rospy.Time.now()

		self.odom_msg = odom_msg


	def ref_cb(self, odom_msg):
		self.ref_msg = odom_msg

if __name__ == '__main__':
	rospy.init_node('bag_data_node')
	node = BagDataNode()

	time_elapsed = Time()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		curr_time = rospy.Time.now()

		if curr_time.to_sec() - node.start_time.to_sec() > 1e-4 and not node.use_model and curr_time.to_sec() - node.start_time.to_sec() >= 30: # 30 seconds
			node.dyn_reconfig_client.update_configuration({"use_model": True})
			node.use_model = True
			print("------- Using model -------")

		if curr_time.to_sec() - node.start_time.to_sec() > 1e-4 and curr_time.to_sec() - node.start_time.to_sec() >= 120:
			node.bag.close()
			os.system("rosnode kill -a && sleep 5 && kill -2 $( ps -C roslaunch -o pid= ) && sleep 2 && kill -2 $( ps -C roscore -o pid= )")
			os.system("tmux kill-session")
			break

		rate.sleep()
		node.bag.write("/downward/vio/odometry", node.odom_msg)
		node.bag.write("/reference_vis", node.ref_msg)
		time_elapsed.data = curr_time - node.start_time
		node.bag.write("/time_elapsed", time_elapsed)

		print("Time elapsed", time_elapsed.data.to_sec())
