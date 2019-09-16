#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosbag

from dynamic_reconfigure.client import Client as DynamicReconfigureClient

from controller_adaptiveclbf.msg import DebugData
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Time

import os
import time

BASE_PATH = '/home/davidfan/Downloads/'
BAG_FILENAME = 'vanilla3'
LEARNING = True

class BagDataNode(object):
	def __init__(self):
		self.bag = rosbag.Bag(BASE_PATH + BAG_FILENAME + '.bag', 'w')

		# dynamic reconfigure client
		self.dyn_reconfig_client = DynamicReconfigureClient('controller_adaptiveclbf_reconfig', timeout=30, config_callback=self.reconfigure_cb)

		if not LEARNING:
			self.dyn_reconfig_client.update_configuration({"model_train": False})
		else:
			self.dyn_reconfig_client.update_configuration({"N_updates": 200})

		self.sub_odom = rospy.Subscriber('/downward/vio/odometry', Odometry, self.odom_cb, queue_size=1)
		self.sub_ref = rospy.Subscriber('/reference_vis', Odometry, self.ref_cb, queue_size=1)
		self.sub_debug = rospy.Subscriber('/adaptive_clbf/debug', DebugData, self.debug_cb, queue_size=1)

		self.odom_msg = Odometry()
		self.ref_msg = Odometry()
		self.debug_msg = DebugData()

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


	def debug_cb(self, debug_msg):
		self.debug_msg = debug_msg

if __name__ == '__main__':
	rospy.init_node('bag_data_node')
	node = BagDataNode()

	time_elapsed = Time()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		curr_time = rospy.Time.now()

		if curr_time.to_sec() - node.start_time.to_sec() > 1e-4 and not node.use_model and curr_time.to_sec() - node.start_time.to_sec() >= 30: # 30 seconds
			if LEARNING:
				# node.dyn_reconfig_client.update_configuration({"model_train": False})
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
		node.bag.write("/adaptive_clbf/debug", node.debug_msg)
		time_elapsed.data = curr_time - node.start_time
		node.bag.write("/time_elapsed", time_elapsed)

		print("Time elapsed", time_elapsed.data.to_sec())
