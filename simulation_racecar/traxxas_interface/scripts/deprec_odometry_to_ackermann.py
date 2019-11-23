#!/usr/bin/env python
# Assume 2D, kinematic bicycle model, and no slip -> Compute steering angle and
# Longitudinal speed of rear axle from vehicle's odometry.

import rospy 
import math
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf.transformations
import tf2_ros

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped

  
def odometry_callback(data):
  
  '''
  # IDEA. Use KBM from  Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design
  global l_f
  global l_r
  # 0. Get yaw angle.
  quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  roll = euler[0]
  pitch = euler[1]
  yaw = euler[2]
  # 1. Compute angle between QSF's x axis and QSF's velocity vector in X-Y -> beta.
  vx_qsf = data.twist.twist.linear.x  # x_qsf is currently pointing backwards.
  vy_qsf = data.twist.twist.linear.y  # y_qsf is currently pointint to the right.
  vx_fw = -vx_qsf
  vy_fw = -vy_qsf
  beta = math.atan2(vy_fw,vx_fw) - yaw # BUG: Not beta, it's beta+psi you are calculating!
  print('YAW: ' + str(math.degrees(yaw)))
  #print('Beta_deg: '+str(math.degrees(beta)))
  # 2. Use (1e) to compute the steering angle -> delta.
  delta = math.atan(math.tan(beta)*(l_f + l_r)/l_r) 
  # 3. Just take the QSF's speed as speed of rear axle.
  v = math.sqrt(math.pow(vx_fw,2) + math.pow(vy_fw,2))
  if math.fabs(beta) > 1.141:
    v = -v
  # Pack steering angle and speed into an Ackermann Message and publish.
  current_ackermann_msg = AckermannDriveStamped()
  current_ackermann_msg.header.stamp = rospy.Time.now() 
  current_ackermann_msg.drive.steering_angle = delta
  current_ackermann_msg.drive.speed = v
  pub.publish(current_ackermann_msg)
  '''
  # 1. Get the most recent transform.
  global tfBuffer

  try:
    current_ackermann_msg = AckermannDriveStamped()
    if running_sim==False:
      trans = tfBuffer.lookup_transform ('base_link', 'map', rospy.Time())

      # 2. Transform the received odometry into a body fixed frame.
      v = Vector3Stamped()
      v.vector.x = data.twist.twist.linear.x
      v.vector.y = data.twist.twist.linear.y
      v.vector.z = data.twist.twist.linear.z
      t = TransformStamped()
      t.transform.rotation.x = trans.transform.rotation.x
      t.transform.rotation.y = trans.transform.rotation.y
      t.transform.rotation.z = trans.transform.rotation.z
      t.transform.rotation.w = trans.transform.rotation.w
      vt = tf2_geometry_msgs.do_transform_vector3(v, t)

      # 3. Compute the speed at this point.
      speed = math.sqrt(math.pow(vt.vector.x,2)+math.pow(vt.vector.y,2))
      # 4. Set steering angle to some default value (not used for now).
      current_ackermann_msg.header.stamp = rospy.Time.now() 
      current_ackermann_msg.drive.steering_angle = 0.0
      current_ackermann_msg.drive.speed = speed
    else:
      a = 1

    pub.publish(current_ackermann_msg)


  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    raise

   
if __name__ == '__main__': 
  try:    
    running_sim = True  # T: Gazebo, F: Real Hardware
    rospy.init_node('odometry_to_ackermann')       
    # TODO: Measure correct values for stampede. 
    #l_f = 0.18 # Stampede: l_f: 18cm, l_f+l_r: 27cm
    #l_r = 0.09

    rospy.Subscriber('/downward/vio/odometry', Odometry, odometry_callback, queue_size=1)
    pub = rospy.Publisher('ackermann_current', AckermannDriveStamped, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass
