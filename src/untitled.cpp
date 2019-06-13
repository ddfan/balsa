while (ros::ok()) {
    ros::spinOnce();

    if ((ros::Time::now() - setpoint_time_stamp_).toSec() >
        setpoint_callback_timeout_duration) {
      x_d_.drive.steering_angle = 0;
      x_d_.drive.steering_angle_velocity = 0;
      x_d_.drive.speed = 0;
      x_d_.drive.acceleration = 0;
      x_d_.drive.jerk = 0;

      ROS_WARN("Controller setpoint callback timeout!!");
    }

    // Create vector 3 with tdotx, tdoty, tdotz.
    tf::Vector3 velocity_map_frame = tf::Vector3(odom_est_.twist.twist.linear.x,
      odom_est_.twist.twist.linear.y,
      odom_est_.twist.twist.linear.z);
    tf::Vector3 velocity_body_frame = tf::Vector3(rotmat_ImuBLf_trp.tdotx(velocity_map_frame),
      rotmat_ImuBLf_trp.tdoty(velocity_map_frame),
      rotmat_ImuBLf_trp.tdotz(velocity_map_frame));

    controller.get_control_input(x_d_, odom_est_, vel_est_encoders_, accel_est_,
                                 output, debug_msg, params_msg, velocity_body_frame);

    /* check callback timeouts.  If no state estimate updates, publish zeros */
    bool callback_timeout = false;
    if ((ros::Time::now() - odom_time_stamp_).toSec() >
        callback_timeout_duration) {
      ROS_ERROR("Odometry estimate callback timeout!");
      callback_timeout = true;
    }

    /* if callback timeout, publish zeros or safely land */
    if (callback_timeout) {
      output.drive.steering_angle = 0;
      output.drive.steering_angle_velocity = 0;
      output.drive.speed = 0;
      output.drive.acceleration = 0;
      output.drive.jerk = 0;
    }

    /* parse out control_input_message and publish to correct topic */
    output.header.stamp = ros::Time::now();
    output_pub.publish(output);

    /* keep track of operation time */
    if (true) {  // condition for vehicle armed and running
      total_operation_time_ += ros::Duration(1.0 / controller_freq);
    }

    std_msgs::Duration dur_msg;
    dur_msg.data = total_operation_time_;
    operation_time_pub.publish(dur_msg);
    debug_msg.total_operation_time = total_operation_time_.toSec();

    /* publish debug */
    debug_msg.header.stamp = ros::Time::now();
    debug_pub.publish(debug_msg);

    /* publish params at a lower rate, every 10 sec. */
    if (loop_rate_downsample > controller_freq * 10) {
      params_pub.publish(params_msg);
      loop_rate_downsample = 0;
    } else {
      loop_rate_downsample++;
    }

    loop_rate.sleep();
  }