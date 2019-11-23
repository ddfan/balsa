#!/usr/bin/env python
import rospy

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped 
from nav_msgs.msg import Path
import math
import rosbag

class TeachNode:
    def __init__(self):
        # init internals
        self.last_saved_odometry = PoseStamped()
        self.first = True
        self.path = Path()

        self.record = True

        # Set subscribers
        rospy.Subscriber('/downward/vio/odometry', Odometry, self.odometry_callback)

        # Set service servers
        save_path_server = rospy.Service('save_path', Trigger, self.save_path)

    def odometry_callback(self, msg):
      path_point_candidate = PoseStamped()
      path_point_candidate.pose = msg.pose.pose
      path_point_candidate.header = msg.header # BUGFIX: Timestamp for replay.

      
      if (self.record==True):
        if self.first == True:
          self.path.poses.append(path_point_candidate)
          self.last_saved_odometry = path_point_candidate
          self.first = False
        else:
          dist_to_last = \
            math.sqrt(math.pow(self.last_saved_odometry.pose.position.x - path_point_candidate.pose.position.x,2)) + \
            math.sqrt(math.pow(self.last_saved_odometry.pose.position.y - path_point_candidate.pose.position.y,2))+ \
            math.sqrt(math.pow(self.last_saved_odometry.pose.position.z - path_point_candidate.pose.position.z,2))
          
          # Check if large enough distance.
          if (dist_to_last >= 1.0):
            print('New Waypoint')
            self.path.poses.append(path_point_candidate)
            self.last_saved_odometry = path_point_candidate
          

    def save_path(self,req):
      self.record == False
      # Save path.
      bag = rosbag.Bag('/home/nikhilesh/traxxas_ws/src/ugv_traxxas/f110/traxxas_interface/teach_paths/waypoints6.bag', 'w')
      for waypoint in self.path.poses: 
        new_waypoint = PoseWithCovarianceStamped()
        new_waypoint.pose.pose = waypoint.pose
        new_waypoint.header = waypoint.header
        bag.write('initialpose', new_waypoint, new_waypoint.header.stamp)

      bag.write('teach_path', self.path)
      bag.close()

      print('Waypoints were saved to ROS bag.')
      print('Length: '+ str(len(self.path.poses)))
      return {'success': True, 'message': 'Waypoints were saved'}

      
# Start the node
if __name__ == '__main__':
    rospy.init_node("teach_node")
    node = TeachNode()
    rospy.spin()
