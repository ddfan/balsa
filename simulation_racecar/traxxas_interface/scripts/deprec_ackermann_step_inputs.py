#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

def talker():
  ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)
  rospy.init_node('pwm_mapping_test_node', anonymous=True)

  # Publish one command.
  test_cmd = AckermannDriveStamped()
  test_cmd.drive.steering_angle = -0.2 # CHANGE HERE
  test_cmd.drive.speed = -1.0  # CHANGE HERE
  ackermann_pub.publish(test_cmd)

  rospy.sleep(1.0)

  # Publish command to stop.
  stop_cmd = AckermannDriveStamped()
  stop_cmd.drive.steering_angle = 0.0
  stop_cmd.drive.speed = 0.0
  ackermann_pub.publish(stop_cmd)

   
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass