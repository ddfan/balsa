#!/usr/bin/env python
# Note: This code is based on the TEB tutorials on the ROS wiki.

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
 
# Speed -> PWM mapping.
def openloop_speed2pwm(speed):
  global P2_MAP_OFFSET_PRM_SET
  global P2_MAXOUT_PRM_SET
  global P2_MAXCMD_PRM_SET
  pwm = (P2_MAXOUT_PRM_SET/P2_MAXCMD_PRM_SET)*speed + P2_MAP_OFFSET_PRM_SET

# PWM -> Speed mapping.
def openloop_pwm2speed(pwm):
  # TODO
  return 0.0

# speed_error->PWM
def speed_pid(speed_error):
  global K_p, K_i, K_d
  pwm_pid = speed_error*K_p 
  pwm_openloop = openloop_speed2pwm(ack_ref_current.drive.speed)
  pwm_command = pwm_pid + pwm_openloop
  return pwm_command

# Save the current reference speed from movebase.
def ack_ref_callback(data):
  global ack_ref_current
  ack_ref_current = data

def ack_curr_callback(data):
  global running_sim
  global ack_ref_current 

  ack_command = AckermannDriveStamped()

  # Only apply PID feedback and PWM mapping when on hardware.
  if running_sim==False:
    # PID for speed.
    error_speed = ack_ref_current.drive.speed - data.drive.speed
    input_pwm = speed_pid(error_speed)
    input_ackermann = openloop_pwm2speed(input_pwm)

    ack_command.header.stamp = rospy.Time.now()
    ack_command.drive.steering_angle = ack_ref_current.drive.steering_angle
    ack_command.drive.speed = input_ackermann
  
  else:
    ack_command = ack_ref_current

  # Publish.
  pub.publish(ack_command)
    
   
if __name__ == '__main__': 
  try:    
    rospy.init_node('ackermann_pid')        

    running_sim = True  # T: Gazebo, F: Real Hardware

    # PWM mapping on the DSP for the vehicle's speed (ToDo: Get this directly from file).
    P2_MAP_OFFSET_PRM_SET = 0.15
    P2_MINCMD_PRM_SET = -10.0
    P2_MAXCMD_PRM_SET = 10.0
    P2_MINOUT_PRM_SET = -0.05
    P2_MAXOUT_PRM_SET = 0.05

    # (Speed) PID gains for feedback only.
    K_p = 0.1
    K_i = 0.0
    K_d = 0.0

    ack_ref_current = AckermannDriveStamped()
    rospy.Subscriber('ackermann_reference', AckermannDriveStamped, ack_ref_callback, queue_size=1)
    rospy.Subscriber('ackermann_current', AckermannDriveStamped, ack_curr_callback, queue_size=1)
    pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    

  except rospy.ROSInterruptException:
    pass
