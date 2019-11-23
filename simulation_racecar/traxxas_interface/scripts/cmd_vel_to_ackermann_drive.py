#!/usr/bin/env python
# Note: This code is based on the TEB tutorials on the ROS wiki.

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0
  radius = v / omega
  return math.atan(wheelbase / radius)
  
def convert_steering_angle_to_rot_vel(v, steering_angle, wheelbase):
  return math.tan(steering_angle)/wheelbase*v

def twist_to_ackermann_callback(data):
  global wheelbase
  global frame_id
  global pub
  global enabled
    
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
     
  msg = AckermannDriveStamped() 
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  if enabled == True:
    msg.drive.steering_angle = steering
    msg.drive.speed = v

  else:
    msg.drive.steering_angle = 0.0
    msg.drive.speed = 0.0

  pub_ackermann.publish(msg)
   

def ackermann_to_twist_callback(data):
  global wheelbase
  global frame_id
  global pub
  global enabled
    
  v = data.drive.speed
  steering = data.drive.steering_angle

  omega = convert_steering_angle_to_rot_vel(v,steering,wheelbase)
     
  msg = Twist() 
  if enabled == True:
    msg.angular.z = omega
    msg.linear.x = v

  else:
    msg.angular.z = 0
    msg.linear.x = 0

  pub_twist.publish(msg)
   
if __name__ == '__main__': 
  try:    
    rospy.init_node('cmd_vel_to_ackermann_drive')        
    wheelbase = 0.48 #0.325 # ToDo: Get this from parameter server.
    enabled =  True
    time_last_OK_sec = 0.0
    frame_id = 'base_link'

    rospy.Subscriber('cmd_vel_out', Twist, twist_to_ackermann_callback, queue_size=1)
    pub_ackermann = rospy.Publisher('ackermann_cmd_openloop', AckermannDriveStamped, queue_size=1)

    rospy.Subscriber('ackermann_cmd_joy', AckermannDriveStamped, ackermann_to_twist_callback, queue_size=1)
    pub_twist = rospy.Publisher('cmd_vel_joy', Twist, queue_size=1)

    rospy.spin()
    

  except rospy.ROSInterruptException:
    pass
