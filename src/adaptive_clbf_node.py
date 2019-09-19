#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
import numpy as np
import tf.transformations as tr
from tf import TransformListener

from dynamic_reconfigure.client import Client as DynamicReconfigureClient

# Import custom message data and dynamic reconfigure variables.
from controller_adaptiveclbf.msg import DebugData

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Empty, Bool, Header
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from adaptive_clbf import AdaptiveClbf

from actionlib_msgs.msg import GoalStatus

class AdaptiveClbfNode(object):
    def __init__(self):
        self.tf = TransformListener()
        self.odom_frame = rospy.get_namespace() + "odom"
        self.base_link_frame = rospy.get_namespace() + "base_link"

        self.adaptive_clbf = AdaptiveClbf(odim=6)
        self.prev_odom_timestamp = rospy.Time(0)
        self.prev_goal_timestamp = rospy.Time(0)
        self.joy_cmd_time = rospy.Time(0)
        self.e_stop_time = rospy.Time(0)
        self.joy_cmd = AckermannDriveStamped()
        self.heartbeat_time = rospy.Time(0)
        self.pointcloud_time = rospy.Time(0)

        self.odom = Odometry()
        self.encoder_odom = Odometry()
        self.use_pose_goal = True

        self.x = np.zeros((4,1))
        self.u_prev = np.zeros((2,1))
        self.z_ref = np.zeros((5,1))
        self.z_ref_dot = np.zeros((5,1))

        self.current_vel_body_x = 0.0
        self.current_vel_body_y = 0.0

        self.enable = True

        self.sent_train_goal = False

        self.params={}
        self.params["vehicle_length"] = rospy.get_param('~vehicle_length',0.5)
        self.params["steering_limit"] = rospy.get_param('~steering_limit',1.0)
        self.params["scale_acceleration"] = rospy.get_param('~scale_acceleration', 1.0)
        self.params["acceleration_gain"] = rospy.get_param('~acceleration_gain', 1.0)
        self.params["acceleration_deadband"] = rospy.get_param('~acceleration_deadband', 0.0)
        self.params["max_accel"] = rospy.get_param('~max_accel',1.0)
        self.params["min_accel"] = rospy.get_param('~min_accel',-1.0)
        self.params["kp_z"] = rospy.get_param('~kp_z',1.0)
        self.params["kd_z"] = rospy.get_param('~kd_z',1.0)
        self.params["clf_epsilon"] = rospy.get_param('~clf_epsilon',1.0)
        self.params["reverse_velocity_goal"] = rospy.get_param('~reverse_velocity_goal',True)

        self.params["learning_verbose"] = rospy.get_param('~learning_verbose',False)
        self.params["use_model"] = rospy.get_param('~use_model',True)
        self.params["add_data"] = rospy.get_param('~add_data',True)
        self.params["model_train"] = rospy.get_param('~model_train',True)
        self.params["check_model"] = rospy.get_param('~check_model',True)
        self.params["measurement_noise"] = rospy.get_param('~measurement_noise', 1.0)
        self.params["N_data"] = rospy.get_param('~N_data',200)
        self.params["max_error"] = rospy.get_param('~max_error',1.0)

        self.params["use_qp"] = rospy.get_param('~use_qp',True)
        self.params["qp_u_cost"] = rospy.get_param('~qp_u_cost',0.01)
        self.params["qp_u_prev_cost"] = rospy.get_param('~qp_u_prev_cost',1.0)
        self.params["qp_p1_cost"] = rospy.get_param('~qp_p1_cost',1.0e6)
        self.params["qp_p2_cost"] = rospy.get_param('~qp_p2_cost',1.0e6)
        self.params["qp_ksig"] = rospy.get_param('~qp_ksig',1.0)
        self.params["qp_max_var"] = rospy.get_param('~qp_max_var',1.0)
        self.params["qp_verbose"] = rospy.get_param('~qp_verbose',False)
        self.params["max_velocity"] = rospy.get_param('~max_velocity',5.0)
        self.params["min_velocity"] = rospy.get_param('~min_velocity',0.001)
        self.params["barrier_vel_gamma"] = rospy.get_param('~barrier_vel_gamma',1.0)
        self.params["use_barrier_vel"] = rospy.get_param('~use_barrier_vel',True)
        self.params["use_barrier_pointcloud"] = rospy.get_param('~use_barrier_pointcloud',True)
        self.params["barrier_radius"] = rospy.get_param('~barrier_radius',1.0)
        self.params["barrier_radius_velocity_scale"] = rospy.get_param('~barrier_radius_velocity_scale',1.0)
        self.params["barrier_pc_gamma_p"] = rospy.get_param('~barrier_pc_gamma_p',1.0)
        self.params["barrier_pc_gamma"] = rospy.get_param('~barrier_pc_gamma',1.0)
        self.params["barrier_max_distance"] = rospy.get_param('~barrier_max_distance',1.0)
        self.params["barrier_resolution"] = rospy.get_param('~barrier_resolution',1.0)
        self.params["verbose"] = rospy.get_param('~verbose',False)
        self.kp_goal = rospy.get_param('~kp_goal',1.0)
        self.desired_vel = rospy.get_param('~desired_vel',0.5)

        # Create a dynamic reconfigure client
        self.dyn_reconfig_client = DynamicReconfigureClient('controller_adaptiveclbf_reconfig', timeout=30, config_callback=self.reconfigure_cb)

        # Create publishers
        self.pub_debug = rospy.Publisher('~debug', DebugData, queue_size=10)
        self.pub_control = rospy.Publisher('output', AckermannDriveStamped, queue_size = 1)
        self.pub_ref = rospy.Publisher('reference_vis', Odometry, queue_size = 1)
        self.pub_odom = rospy.Publisher('odom_vis', Odometry, queue_size = 1)

        # Create subscribers
        rospy.Subscriber('pose_target', PoseStamped, self.pose_goal_cb, queue_size=1)
        rospy.Subscriber('odometry_target', Odometry, self.odometry_goal_cb, queue_size=1)
        rospy.Subscriber('odometry', Odometry, self.odometry_cb, queue_size=1)
        rospy.Subscriber('encoder/odom', Odometry, self.encoders_cb, queue_size=1)
        rospy.Subscriber('scan', LaserScan, self.pointcloud_cb, queue_size=1)
        rospy.Subscriber('joy_cmd', AckermannDriveStamped, self.joy_cmd_cb, queue_size = 1)
        rospy.Subscriber('heartbeat_base', Empty, self.heartbeat_cb, queue_size = 1)
        rospy.Subscriber('e_stop', Bool, self.e_stop_cb, queue_size = 1)

        # training timer
        rate = rospy.get_param('~rate', 30.0)
        self.dt = 1.0 / rate
        self.params["dt"] = self.dt
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_cb)

        self.odom_cb_called = False
        self.reset_ref = True
        self.cbf_pointcloud_list=[]
        # Initialize message variables.
        self.enable = rospy.get_param('~enable', True)

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher."""
        self.pub_debug = rospy.Publisher('~debug', DebugData, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub_debug.unregister()

    def pose_goal_cb(self,goal):
        x_des = 0
        y_des = 0
        desired_vel = 0
        desired_heading = 0

        if "base_link" in goal.header.frame_id:
            q = (goal.pose.orientation.x,
                 goal.pose.orientation.y,
                 goal.pose.orientation.z,
                 goal.pose.orientation.w)
            euler = tr.euler_from_quaternion(q)
            rel_yaw = euler[2]

            q2 = (self.odom.pose.pose.orientation.x,
                 self.odom.pose.pose.orientation.y,
                 self.odom.pose.pose.orientation.z,
                 self.odom.pose.pose.orientation.w)
            euler2 = tr.euler_from_quaternion(q2)
            curr_yaw = euler2[2]

            desired_heading = curr_yaw + rel_yaw

            dx = np.cos(curr_yaw) * goal.pose.position.x - np.sin(curr_yaw) * goal.pose.position.y
            dy = np.sin(curr_yaw) * goal.pose.position.x + np.cos(curr_yaw) * goal.pose.position.y

            x_des = self.odom.pose.pose.position.x + dx
            y_des = self.odom.pose.pose.position.y + dy

        else:
            # get current desired state
            q = (goal.pose.orientation.x,
                 goal.pose.orientation.y,
                 goal.pose.orientation.z,
                 goal.pose.orientation.w)
            euler = tr.euler_from_quaternion(q)
            desired_heading = euler[2]
            x_des = goal.pose.position.x
            y_des = goal.pose.position.y

        # scale desired velocity if goal is a fixed position
        if self.kp_goal > 0:
            distance_from_goal = np.sqrt((goal.pose.position.x-self.odom.pose.pose.position.x)**2 + (goal.pose.position.y-self.odom.pose.pose.position.y)**2 )
            desired_vel = np.minimum(self.desired_vel,distance_from_goal * self.kp_goal)
        else:
            desired_vel = self.desired_vel

        if self.params["reverse_velocity_goal"] and self.current_vel_body_x < 0:
            desired_vel = - np.abs(desired_vel)

        x_ref = np.array([[x_des,y_des,desired_heading,desired_vel]]).T
        self.z_ref = self.adaptive_clbf.dyn.convert_x_to_z(x_ref)
        self.z_ref_dot = np.zeros((5,1))

        self.prev_goal_timestamp = goal.header.stamp

    def odometry_goal_cb(self,goal):
        dt = (goal.header.stamp - self.prev_goal_timestamp).to_sec()

        if dt < 0:
            rospy.logwarn("detected jump back in time!  resetting prev_goal_timestamp.")
            self.prev_goal_timestamp = goal.header.stamp
            return

        if dt < self.dt / 4.0:
            rospy.logwarn("dt is too small! (%f)  skipping this goal callback!", dt)
            return

        # get current desired state
        q = (goal.pose.pose.orientation.x,
             goal.pose.pose.orientation.y,
             goal.pose.pose.orientation.z,
             goal.pose.pose.orientation.w)
        euler = tr.euler_from_quaternion(q)
        desired_heading = euler[2]
        desired_vel = goal.twist.twist.linear.x

        x_ref = np.array([[goal.pose.pose.position.x,goal.pose.pose.position.y,desired_heading,desired_vel]]).T
        z_ref_new = self.adaptive_clbf.dyn.convert_x_to_z(x_ref)
        self.z_ref_dot = (z_ref_new - self.z_ref) / dt
        self.z_ref = z_ref_new
        self.prev_goal_timestamp = goal.header.stamp

    def timer_cb(self,_event):
        if self.params["model_train"]:

            if not self.sent_train_goal:
                self.train_start_time = rospy.get_rostime()
                print("sending training goal")
                self.adaptive_clbf.train_model_action_client.send_goal(self.adaptive_clbf.train_model_goal)
                self.sent_train_goal = True
                return

            # states 0 = PENDING, 1 = ACTIVE, 2 = PREEMPTED, 3 = SUCCEEDED
            state = self.adaptive_clbf.train_model_action_client.get_state()
            # print("State:",state)
            if self.sent_train_goal and (state == GoalStatus.PENDING or state == GoalStatus.ACTIVE):
                return
            elif state == GoalStatus.SUCCEEDED:
                result = self.adaptive_clbf.train_model_action_client.get_result() 
                if hasattr(result, 'model_trained'):
                    self.adaptive_clbf.model_trained = self.adaptive_clbf.train_model_action_client.get_result().model_trained
                else:
                    self.adaptive_clbf.model_trained = False

            self.sent_train_goal = False

            end_time = rospy.get_rostime()
            # if self.params["verbose"]:
            rospy.logwarn(["training latency (ms): ", (end_time-self.train_start_time).to_sec() * 1000.0])

    def joy_cmd_cb(self, ackermann_drive_msg):
        self.joy_cmd = ackermann_drive_msg
        if self.joy_cmd.drive.speed == 0:
            self.joy_cmd.drive.jerk = -100
        self.joy_cmd_time = rospy.get_rostime()
        rospy.logwarn("Detected joystick override!")

    def heartbeat_cb(self, msg):
        self.heartbeat_time = rospy.get_rostime()

    def e_stop_cb(self, msg):
        if msg.data:
            self.e_stop_time = rospy.get_rostime()

    def encoders_cb(self,encoder_odom):
        self.encoder_odom = encoder_odom

    def pointcloud_cb(self,pointcloud):
        # get current state
        self.pointcloud_time = rospy.get_rostime()

        if not self.params["use_barrier_pointcloud"]:
            return

        q = (self.odom.pose.pose.orientation.x,
             self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z,
             self.odom.pose.pose.orientation.w)
        euler = tr.euler_from_quaternion(q)
        current_heading = euler[2]

        x_curr = self.odom.pose.pose.position.x
        y_curr = self.odom.pose.pose.position.y

        angles = np.arange(pointcloud.angle_min,pointcloud.angle_max-pointcloud.angle_increment,pointcloud.angle_increment) + current_heading
        ranges = np.array(pointcloud.ranges)
        angles = angles[np.isfinite(ranges)]
        ranges = ranges[np.isfinite(ranges)]

        x = np.cos(angles) * np.array(ranges) + x_curr
        y = np.sin(angles) * np.array(ranges) + y_curr

        # Downsample pointcloud
        barrier_max_distance = self.params["barrier_max_distance"]
        barrier_resolution = self.params["barrier_resolution"]
        if (x.size > 0):
            x_min = -barrier_max_distance
            x_max = barrier_max_distance
            y_min = -barrier_max_distance
            y_max = barrier_max_distance
            x_filtered = [x[0]]
            y_filtered = [y[0]]
            for i in range(1, len(x)):
                # Remove points that are too close to each other
                if np.sqrt((x[i] - x_filtered[-1])**2 + (y[i] - y_filtered[-1])**2) > barrier_resolution:
                    # Remove points that are far away from the robot
                    if (x[i] - x_curr < x_max) and (x[i] - x_curr > x_min) and (y[i] - y_curr < y_max) and (y[i] - y_curr > y_min) :
                        x_filtered.append(x[i])
                        y_filtered.append(y[i])
            x = np.array(x_filtered)
            y = np.array(y_filtered)
            # update barriers
            curr_vel = np.sqrt(self.odom.twist.twist.linear.x ** 2 + self.odom.twist.twist.linear.y ** 2)
            radius = curr_vel * self.params["barrier_radius_velocity_scale"]
            radius = np.maximum(radius,self.params["barrier_radius"])
            self.adaptive_clbf.update_barrier_locations(x=x,y=y,radius=radius)

    def odometry_cb(self, odom):
        start_time = rospy.get_rostime()

        # TODO:  theory - maybe dt of rostime and header does not match dt of actual odometry data.  this might cause big problems.
        dt = (odom.header.stamp - self.prev_odom_timestamp).to_sec()

        if dt < 0:
            rospy.logwarn("detected jump back in time!  resetting prev_odom_timestamp.")
            self.prev_odom_timestamp = self.odom.header.stamp
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_cb)
            self.sent_train_goal = False
            return

        if dt < self.dt:
            # rospy.logwarn("dt is too small! (%f)  skipping this odometry callback!", dt)
            return

        # get current odom in base_link frame
        try:
            t = self.tf.getLatestCommonTime(self.odom_frame, self.base_link_frame)
            position, quaternion = self.tf.lookupTransform(self.odom_frame, self.base_link_frame, t)
            hdr = copy.copy(odom.header)
            hdr.frame_id = odom.child_frame_id
            camera_to_bl = self.tf.asMatrix(self.base_link_frame,hdr)
        except:
            raise
            return

        self.odom.header = odom.header
        self.odom.child_frame_id = self.base_link_frame
        self.odom.pose.pose.position.x = position[0]
        self.odom.pose.pose.position.y = position[1]
        self.odom.pose.pose.position.z = position[2]
        self.odom.pose.pose.orientation.x = quaternion[0]
        self.odom.pose.pose.orientation.y = quaternion[1]
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]

        twist_rot = np.array([odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z])
        twist_vel = np.array([odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z])

        out_rot = np.matmul(camera_to_bl[0:3,0:3], twist_rot)
        out_vel = np.matmul(camera_to_bl[0:3,0:3], twist_vel) + np.cross(camera_to_bl[0:3,-1],out_rot)

        self.odom.twist.twist.linear.x = out_vel[0]
        self.odom.twist.twist.linear.y = out_vel[1]
        self.odom.twist.twist.linear.z = out_vel[2]
        self.odom.twist.twist.angular.x = out_rot[0]
        self.odom.twist.twist.angular.y = out_rot[1]
        self.odom.twist.twist.angular.z = out_rot[2]

        self.prev_odom_timestamp = odom.header.stamp

        if not self.enable:
            return

        # get current state and observations
        q = (self.odom.pose.pose.orientation.x,
             self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z,
             self.odom.pose.pose.orientation.w)
        euler = tr.euler_from_quaternion(q)
        current_roll = euler[0]
        current_pitch = euler[1]
        current_heading = euler[2]
        self.current_vel_body_x = np.cos(current_heading) * self.odom.twist.twist.linear.x + np.sin(current_heading) * self.odom.twist.twist.linear.y
        self.current_vel_body_y = -np.sin(current_heading) * self.odom.twist.twist.linear.x + np.cos(current_heading) * self.odom.twist.twist.linear.y
        
        # self.current_vel_body_x = self.odom.twist.twist.linear.x
        # self.current_vel_body_y = self.odom.twist.twist.linear.y
        
        self.x=np.array([[self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,current_heading,self.current_vel_body_x]]).T
        self.obs = np.array([[current_heading,
                            self.current_vel_body_y,
                            self.odom.twist.twist.angular.z,
                            self.u_prev[0],
                            self.u_prev[1],
                            current_roll,
                            current_pitch
                            # self.encoder_odom.twist.twist.linear.x,
                            # self.encoder_odom.twist.twist.angular.z
                            ]],dtype=np.float32).T

        self.z = self.adaptive_clbf.dyn.convert_x_to_z(self.x)

        add_data = self.params["add_data"]
        # don't add data if no motion
        if (self.current_vel_body_x**2 + self.current_vel_body_y**2) < 0.01:
            add_data = False
        # don't add data if moving backwards
        if self.current_vel_body_x < -0.1:
            add_data = False
        # dont' add first timestep of data
        if not self.odom_cb_called:
            add_data = False
            self.odom_cb_called = True
        # clear barriers if pointcloud callback is too long ago.
        if (rospy.get_rostime() - self.pointcloud_time).to_sec() > 1.0:
            rospy.logwarn("Clearing barriers because no points recieved!")
            self.adaptive_clbf.update_barrier_locations(x=np.array([]),y=np.array([]),radius=0)

        # get control!
        u = self.adaptive_clbf.get_control(self.z,self.z_ref,self.z_ref_dot,dt=dt,obs=self.obs,use_model=self.params["use_model"],add_data=add_data,check_model=self.params["check_model"],use_qp=self.params["use_qp"])
        self.x_ref = self.adaptive_clbf.dyn.convert_z_to_x(self.z_ref)

        # nan check
        if np.isnan(u).any():
            u = np.array([0.0,0.0])

        self.u_prev = copy.copy(u)

        # make message
        u_msg = AckermannDriveStamped()
        u_msg.drive.steering_angle = u[0]

        if self.params["scale_acceleration"] == 0.0:
            # directly apply acclerations
            if u[1] > 0:
                u_msg.drive.acceleration = u[1] * self.params["acceleration_gain"] + self.params["acceleration_deadband"]
            else:
                u_msg.drive.acceleration = u[1] * self.params["acceleration_gain"] - self.params["acceleration_deadband"]
            
        else:
            # or, assume underlying velocity controller
            u_msg.drive.speed = self.current_vel_body_x + self.params["scale_acceleration"] * u[1]

        u_msg.header.stamp = self.odom.header.stamp
        if (rospy.get_rostime() - self.heartbeat_time).to_sec() > 2.0: # or (rospy.get_rostime() - self.e_stop_time).to_sec() < 1.0:
            rospy.logwarn("Publishing zero command, heartbeat lost or e_stopped.")
            zero_msg = AckermannDriveStamped()
            zero_msg.drive.jerk = -100.0
            zero_msg.header.stamp = self.odom.header.stamp
            self.pub_control.publish(zero_msg)
        elif (rospy.get_rostime() - self.joy_cmd_time).to_sec() < 0.5:
            self.joy_cmd.header.stamp = self.odom.header.stamp
            self.pub_control.publish(self.joy_cmd)
            rospy.logwarn("Publishing joystick override command.")
        else:
            self.pub_control.publish(u_msg)

        end_time = rospy.get_rostime()

        # publish reference pose for visualization
        x_ref_msg = Odometry()
        x_ref_msg.header.frame_id = self.odom_frame
        x_ref_msg.header.stamp = self.odom.header.stamp
        x_ref_msg.pose.pose.position.x = self.x_ref[0]
        x_ref_msg.pose.pose.position.y = self.x_ref[1]
        x_ref_msg.pose.pose.position.z = 0.0
        x_ref_msg.pose.pose.orientation.x = 0.0
        x_ref_msg.pose.pose.orientation.y = 0.0
        x_ref_msg.pose.pose.orientation.z = np.sin(self.x_ref[2]/2.0)
        x_ref_msg.pose.pose.orientation.w = np.cos(self.x_ref[2]/2.0)
        # cov = self.adaptive_clbf.sigma
        # x_ref_msg.pose.covariance[0] = cov[0]
        # x_ref_msg.pose.covariance[7] = cov[1]
        x_ref_msg.pose.covariance[0] = self.adaptive_clbf.predict_error
        x_ref_msg.pose.covariance[7] = self.adaptive_clbf.predict_error
        self.pub_ref.publish(x_ref_msg)

        self.pub_odom.publish(self.odom)

        cb_latency = (end_time-start_time).to_sec() * 1000.0
        if self.params["verbose"]:
            rospy.logwarn(["callback latency (ms): ", cb_latency])

        # publish debug data
        debug_msg = DebugData()
        debug_msg.header.stamp = self.odom.header.stamp
        debug_msg.latency = cb_latency
        debug_msg.dt = dt
        debug_msg.heading = current_heading
        debug_msg.vel_x_body = self.current_vel_body_x
        debug_msg.vel_y_body = self.current_vel_body_y
        debug_msg.odom = self.odom

        debug_msg.z = self.adaptive_clbf.debug["z"]
        debug_msg.z_ref = self.adaptive_clbf.debug["z_ref"]
        debug_msg.z_dot = self.adaptive_clbf.debug["z_dot"]
        debug_msg.y_out = self.adaptive_clbf.debug["y_out"]
        debug_msg.mu_rm = self.adaptive_clbf.debug["mu_rm"]
        debug_msg.mu_pd = self.adaptive_clbf.debug["mu_pd"]
        debug_msg.mu_ad = self.adaptive_clbf.debug["mu_ad"]
        debug_msg.rho = self.adaptive_clbf.debug["rho"]
        debug_msg.mu_qp = self.adaptive_clbf.debug["mu_qp"]
        debug_msg.mu_model = self.adaptive_clbf.debug["mu_model"]
        debug_msg.mu = self.adaptive_clbf.debug["mu"]
        debug_msg.u_new = self.adaptive_clbf.debug["u_new"]
        debug_msg.u_unsat = self.adaptive_clbf.debug["u_unsat"]
        debug_msg.trueDelta = self.adaptive_clbf.debug["trueDelta"]
        debug_msg.true_predict_error = self.adaptive_clbf.debug["true_predict_error"]
        debug_msg.mDelta = self.adaptive_clbf.debug["mDelta"]
        debug_msg.sigDelta = self.adaptive_clbf.debug["sigDelta"]
        self.pub_debug.publish(debug_msg)

    def reconfigure_cb(self, config):
        """Create a callback function for the dynamic reconfigure client."""
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.params["vehicle_length"] = config["vehicle_length"]
        self.params["steering_limit"] = config["steering_limit"]
        self.params["scale_acceleration"] = config["scale_acceleration"]
        self.params["acceleration_gain"] = config["acceleration_gain"]
        self.params["acceleration_deadband"] = config["acceleration_deadband"]
        self.params["max_accel"] = config["max_accel"]
        self.params["min_accel"] = config["min_accel"]
        self.params["kp_z"] = config["kp_z"]
        self.params["kd_z"] = config["kd_z"]
        self.params["clf_epsilon"] = config["clf_epsilon"]
        self.params["learning_verbose"] = config["learning_verbose"]
        self.params["measurement_noise"] = config["measurement_noise"]
        self.params["N_data"] = config["N_data"]
        self.params["max_error"] = config["max_error"]
        self.params["use_qp"] = config["use_qp"]
        self.params["qp_u_cost"] = config["qp_u_cost"]
        self.params["qp_u_prev_cost"] = config["qp_u_prev_cost"]
        self.params["qp_p1_cost"] = config["qp_p1_cost"]
        self.params["qp_p2_cost"] = config["qp_p2_cost"]
        self.params["qp_ksig"] = config["qp_ksig"]
        self.params["qp_max_var"] = config["qp_max_var"]
        self.params["qp_verbose"] = config["qp_verbose"]
        self.params["max_velocity"] = config["max_velocity"]
        self.params["min_velocity"] = config["min_velocity"]
        self.params["barrier_vel_gamma"] = config["barrier_vel_gamma"]
        self.params["use_barrier_vel"] = config["use_barrier_vel"]
        self.params["use_barrier_pointcloud"] = config["use_barrier_pointcloud"]
        self.params["barrier_radius"] = config["barrier_radius"]
        self.params["barrier_radius_velocity_scale"] = config["barrier_radius_velocity_scale"]
        self.params["barrier_pc_gamma_p"] = config["barrier_pc_gamma_p"]
        self.params["barrier_pc_gamma"] = config["barrier_pc_gamma"]
        self.params["barrier_max_distance"] = config["barrier_max_distance"]
        self.params["barrier_resolution"] = config["barrier_resolution"]
        self.params["verbose"] = config["verbose"]
        self.params["use_model"] = config["use_model"]
        self.params["model_train"] = config["model_train"]
        self.params["add_data"] = config["add_data"]
        self.params["check_model"] = config["check_model"]
        self.params["reverse_velocity_goal"] = config["reverse_velocity_goal"]
        self.kp_goal = config["kp_goal"]
        self.desired_vel = config["desired_vel"]

        rate = config["rate"]
        self.dt = 1.0 / rate
        self.params["dt"] = self.dt

        self.adaptive_clbf.update_params(self.params)

        # Check to see if node should be started or stopped.
        if self.enable != config["enable"]:
            if config["enable"]:
                self.start()
            else:
                self.stop()
        self.enable = config["enable"]

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('controller_adaptiveclbf')
    # Go to class functions that do all the heavy lifting.
    try:
        AdaptiveClbfNode()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
