#include <controller_xmaxx/DebugData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <iomanip>
#include <tuple>
#include "mavros_msgs/ActuatorControl.h"
#include "mavros_msgs/RCOut.h"
#include "mobility_msgs/PositionTargetMode.h"

typedef mobility_msgs::PositionTargetMode Goal;

// Integrator
struct Integrator {
  double value;

  // Constructors:
  Integrator() { value = 0; }

  // Methods
  void increment(double inc, double dt) {
    value += inc * dt;
  }  // value += inc * dt; }
  void reset() { value = 0; }
};

/* Template for controller that takes in a position target and current
pose/twist estimates
and produces a control input message which can be of various message types.
Return type is ControlInputMessage which is a struct that contains an enum
indicating
the message type along with the message itself.  Use update_attitude_target or
update_<message type> functions within the child class to build the return
struct.*/
class PositionController {
  /* Define config params */
 protected:
  double mass_ = 2.0, max_thrust_rating_ = 60.0, controller_freq_ = 50;
  const double pi_ = 3.1415962;
  bool in_offboard_ = false;

 public:
  PositionController() { load_params_common(); }

  virtual bool get_control_input(
      const mobility_msgs::PositionTargetMode& x_d,
      const mavros_msgs::RCOut& rc, const nav_msgs::Odometry& odom_est,
      const geometry_msgs::TwistStamped& vel_est_encoders,
      const geometry_msgs::WrenchStamped& accel_est,
      mavros_msgs::ActuatorControl& actuator_control,
      controller_xmaxx::DebugData& debug_data,
      controller_xmaxx::ParamsData& params_data) = 0;

  void setOffboard(const bool in_offboard) { in_offboard_ = in_offboard; }

  void load_params_common() {
    /* Read ros params, if not available then set to initialized value */
    ros::NodeHandle nhp("~");
    nhp.param<double>("mass", mass_, mass_);
    nhp.param<double>("max_thrust_rating", max_thrust_rating_,
                      max_thrust_rating_);
    nhp.param<double>("controller_freq", controller_freq_, controller_freq_);
  }

 protected:
  bool rc_has_throttle(const mavros_msgs::RCOut& rc) {
    /*check if first 4 values have value of 900*/
    if (rc.channels.size() > 0 && rc.channels[0] != 900 &&
        rc.channels[0] != 900 && rc.channels[0] != 900 && rc.channels[0] != 900)
      return true;
    else
      return false;
  }

  // TODO: Remove pass by reference version
  void saturate(double& val, const double& max, const double& min) {
    val = (val > max) ? max : val;
    val = (val < min) ? min : val;
  }

  double saturate_by_value(double val, const double& max, const double& min) {
    val = (val > max) ? max : val;
    val = (val < min) ? min : val;
    return val;
  }

  void qProd(Eigen::Quaternion<double>& c, const Eigen::Quaternion<double>& a,
             const Eigen::Quaternion<double>& b) {
    c.w() = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
    c.x() = a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y();
    c.y() = a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x();
    c.z() = a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w();
  }

  double sinc(const double x) {
    if (x == 0) {
      return 1;
    }
    return sin(x) / x;
  }

  double dist_s1(double ang1, double ang2) {
    double ang = ang1 - ang2;
    ang += (ang > pi_) ? -2 * pi_ : (ang < -pi_) ? 2 * pi_ : 0;
    return ang;
  }
  double unwrap(double ang) {
    ang += pi_;
    ang = fmod(ang, 2 * pi_);
    ang -= pi_;
  }

  void get_rpy_from_pose(geometry_msgs::Quaternion q, double& r, double& p,
                         double& y) {
    /* Get rpy angles from x_est quaternions */
    tf::Quaternion q0(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(q0);
    m.getRPY(r, p, y);

    // temporary hack to convert pitch to +-360 degrees, relying on roll>pi/2
    if (r > pi_ / 2 || r < -pi_ / 2) {
      p = ((p > 0) ? 1 : -1) * (pi_ - fabs(p));
    }
    /* If rollocopter is upside down, flip yaw and roll. This is critical
        for avoiding uncontrolled pitching around and around  */
    if (p > pi_ / 2 || p < -pi_ / 2) {
      r = unwrap(r + pi_);
      y = unwrap(y + pi_);
    }
  }
};

/* Direct control of actuator moments using ActuatorControl message.
   Takes in a desired position in world frame and finds a forward velocity
and yaw to move towards the desired position. */
class RollingControllerDirect : public PositionController {
  double controller_freq_ = 50, thrust_max_ = 0.45, thrust_min_ = 0.01,
         kp_pos_ = 1.0, kd_pos_ = 1.0, ki_pos_ = 1.0, kp_vel_ = 1.0,
         kd_vel_ = 0.5, ki_vel_ = 0.0, kp_yaw_ = 1.0, ki_yaw_ = 0.0,
         kd_yaw_ = 2.0, max_vel_error_ = 5.0, max_yaw_error_ = 3.14,
         max_posI_ = 10.0, max_velI_ = 10.0, max_yawI_ = 10.0,
         max_pitch_ = 3.14, COM_length_ = 0.1, kp_pitch_ = 2.0, kd_pitch_ = 4.0,
         moment_roll_scale_ = 10.0, moment_pitch_scale_ = 10.0,
         moment_yaw_scale_ = 10.0, max_pitchI_ = 10.0, max_pitch_error_ = 5.0,
         ki_pitch_ = 0.0, max_velocity_ = 0.1, deadzone_radius_ = 0.1,
         pitch_offset_ = 0.0, point_and_shoot_angle_ = pi_ / 6,
         max_yaw_rate_error_ = 3, max_pitch_rate_error_ = 3;

  int pitch_testing_mode_ = 0, yaw_testing_mode_ = 0,
      velocity_testing_mode_ = 0;

  bool using_encoders_ = false;

 public:
  RollingControllerDirect() : PositionController() {
    load_params();
    posI = Integrator();
    velI = Integrator();
    yawI = Integrator();
    pitchI = Integrator();
    yaw_deadzone_target = 0;
    in_deadzone = false;
  }

  void load_params() {
    /* Read ros params, if not available then set to initialized value */
    ros::NodeHandle nhp("~");
    nhp.param<double>("rolling_kp_pos", kp_pos_, kp_pos_);
    nhp.param<double>("rolling_kd_pos", kd_pos_, kd_pos_);
    nhp.param<double>("rolling_ki_pos", ki_pos_, ki_pos_);
    nhp.param<double>("rolling_kp_vel", kp_vel_, kp_vel_);
    nhp.param<double>("rolling_kd_vel", kd_vel_, kd_vel_);

    nhp.param<double>("rolling_ki_vel", ki_vel_, ki_vel_);
    nhp.param<double>("rolling_kp_yaw", kp_yaw_, kp_yaw_);
    nhp.param<double>("rolling_ki_yaw", ki_yaw_, ki_yaw_);
    nhp.param<double>("rolling_kd_yaw", kd_yaw_, kd_yaw_);
    nhp.param<double>("rolling_thrust_max", thrust_max_, thrust_max_);
    nhp.param<double>("rolling_thrust_min", thrust_min_, thrust_min_);
    nhp.param<double>("controller_freq", controller_freq_, controller_freq_);
    nhp.param<double>("rolling_max_vel_error", max_vel_error_, max_vel_error_);
    nhp.param<double>("rolling_max_yaw_error", max_yaw_error_, max_yaw_error_);
    nhp.param<double>("rolling_max_pitch_error", max_pitch_error_,
                      max_pitch_error_);
    nhp.param<double>("rolling_max_yaw_rate_error", max_yaw_rate_error_,
                      max_yaw_rate_error_);
    nhp.param<double>("rolling_max_pitch_rate_error", max_pitch_rate_error_,
                      max_pitch_rate_error_);
    nhp.param<double>("rolling_max_posI", max_posI_, max_posI_);
    nhp.param<double>("rolling_max_velI", max_velI_, max_velI_);
    nhp.param<double>("rolling_max_yawI", max_yawI_, max_yawI_);
    nhp.param<double>("rolling_max_pitchI", max_pitchI_, max_pitchI_);
    nhp.param<double>("rolling_max_pitch", max_pitch_, max_pitch_);
    nhp.param<double>("rolling_COM_length", COM_length_, COM_length_);
    nhp.param<double>("rolling_kp_pitch", kp_pitch_, kp_pitch_);
    nhp.param<double>("rolling_kd_pitch", kd_pitch_, kd_pitch_);
    nhp.param<double>("rolling_ki_pitch", ki_pitch_, ki_pitch_);

    nhp.param<double>("rolling_moment_roll_scale", moment_roll_scale_,
                      moment_roll_scale_);
    nhp.param<double>("rolling_moment_pitch_scale", moment_pitch_scale_,
                      moment_pitch_scale_);
    nhp.param<double>("rolling_moment_yaw_scale", moment_yaw_scale_,
                      moment_yaw_scale_);
    nhp.param<double>("rolling_max_velocity", max_velocity_, max_velocity_);
    nhp.param<double>("rolling_pitch_offset", pitch_offset_, pitch_offset_);
    nhp.param<double>("rolling_point_and_shoot_angle", point_and_shoot_angle_,
                      point_and_shoot_angle_);
    nhp.param<double>("deadzone_radius", deadzone_radius_, deadzone_radius_);
    nhp.param<int>("pitch_testing_mode", pitch_testing_mode_,
                   pitch_testing_mode_);
    nhp.param<int>("yaw_testing_mode", yaw_testing_mode_, yaw_testing_mode_);
    nhp.param<int>("velocity_testing_mode", velocity_testing_mode_,
                   velocity_testing_mode_);
    nhp.param<bool>("using_encoders", using_encoders_, using_encoders_);
  }

  bool get_control_input(const mobility_msgs::PositionTargetMode& x_d,
                         const mavros_msgs::RCOut& rc,
                         const nav_msgs::Odometry& odom_est,
                         const geometry_msgs::TwistStamped& vel_est_encoders,
                         const geometry_msgs::WrenchStamped& accel_est,
                         mavros_msgs::ActuatorControl& actuator_control,
                         controller_xmaxx::DebugData& debug_data,
                         controller_xmaxx::ParamsData& params_data) {
    double roll_est, pitch_est, yaw_est;
    get_rpy_from_pose(odom_est.pose.pose.orientation, roll_est, pitch_est,
                      yaw_est);

    double roll_rate_est = odom_est.twist.twist.angular.x;
    double pitch_rate_est = odom_est.twist.twist.angular.y;

    double yaw_rate_est;
    if (using_encoders_) {
      yaw_rate_est = vel_est_encoders.twist.angular.z;
    } else {
      yaw_rate_est = odom_est.twist.twist.angular.z;
    }

    double yaw_des = 0.0;
    double desired_velocity = 0, e_yaw = 0, de_yaw = 0;

    /* odom msg has velocity in base_link frame, transform to map */
    Eigen::Vector3d vel_est(odom_est.twist.twist.linear.x,
                            odom_est.twist.twist.linear.y,
                            odom_est.twist.twist.linear.z);
    Eigen::Quaterniond temp_q(
        odom_est.pose.pose.orientation.w, odom_est.pose.pose.orientation.x,
        odom_est.pose.pose.orientation.y, odom_est.pose.pose.orientation.z);
    Eigen::Matrix3d Rot = temp_q.toRotationMatrix();
    vel_est = Rot * vel_est;

    /* Decide which control mode to use based on x_d.type_mask */
    if ((x_d.type_mask & Goal::IGNORE_PX) == Goal::IGNORE_PX &&
        (x_d.type_mask & Goal::IGNORE_PY) == Goal::IGNORE_PY) {
      if (x_d.type_mask == 4095)  // goal reached Ignore everything!
      {
        if (~in_deadzone) {
          /* Entering deadzone for the first time, set deadzone target. */
          in_deadzone = true;
          yaw_deadzone_target = yaw_est;
        }
        e_yaw = dist_s1(yaw_deadzone_target, yaw_est);
        de_yaw = 0;
        desired_velocity = 0;
      } else {
        if (in_deadzone) {
          /* Exiting deadzone */
          in_deadzone = false;
        }
        desired_velocity = x_d.velocity.x;
        // e_yaw = atan2(x_d.velocity.y, x_d.velocity.x);
        e_yaw = x_d.yaw;  // relative heading
        ///* target is behind, move backwards */
        // if (fabs(e_yaw) > pi_ / 2 + 0.2)
        //  e_yaw = unwrap(e_yaw + pi_);
        de_yaw = x_d.yaw_rate - yaw_rate_est;
      }

      /* Expect x_d.position and x_d.velocity in the world frame.  need to
         convert to
          body/rolling frame based on current yaw estimate */
    } else {
      double e_x = x_d.position.x - odom_est.pose.pose.position.x;
      double e_y = x_d.position.y - odom_est.pose.pose.position.y;
      double de_x = x_d.velocity.x - vel_est(0);
      double de_y = 0 - vel_est(1);
      double e_dist = sqrt(pow(e_x, 2) + pow(e_y, 2));

      /* Calculate desired acceleration in body_frame */
      double e_x_body = cos(yaw_est) * e_x + sin(yaw_est) * e_y;
      // e_y_body = -sin(yaw_est) * e_x + cos(yaw_est) * e_y;
      double de_x_body = cos(yaw_est) * de_x + sin(yaw_est) * de_y;

      /* determine if target point is in front or behind */
      double target_heading_abs = atan2(e_y, e_x);
      double target_heading_rel = dist_s1(target_heading_abs, yaw_est);
      if (e_dist < deadzone_radius_) {
        if (~in_deadzone) {
          /* Entering deadzone for the first time, set deadzone target. */
          in_deadzone = true;
          yaw_deadzone_target = yaw_est;
        }
        yaw_des = yaw_deadzone_target;
      } else if (fabs(target_heading_rel) < pi_ / 2) {
        if (in_deadzone) {
          /* Exiting deadzone */
          in_deadzone = false;
        }
        /* target is ahead, move forward */
        yaw_des = target_heading_abs;
      } else {
        if (in_deadzone) {
          /* Exiting deadzone */
          in_deadzone = false;
        }
        /* target is behind, move backwards */
        yaw_des = unwrap(target_heading_abs + pi_);
      }

      if (yaw_testing_mode_) {
        ROS_WARN("Enabled Yaw Testing Mode");
        yaw_des = x_d.yaw;  // for TESTING purposes
      }
      e_yaw = dist_s1(yaw_des, yaw_est);
      de_yaw = x_d.yaw_rate - yaw_rate_est;

      /* Postion Controller.  Cascade to velocity later. */
      desired_velocity = kp_pos_ * e_x_body + kd_pos_ * de_x_body;
      if (velocity_testing_mode_) {
        ROS_WARN("Enabled Velocity Testing Mode");
        desired_velocity = x_d.velocity.x;  // for TESTING purposes
      }
    }

    double current_velocity_body_x, current_velocity_body_y;
    if (using_encoders_) {
      current_velocity_body_x = vel_est_encoders.twist.linear.x;
      current_velocity_body_y = vel_est_encoders.twist.linear.y;
    } else {
      current_velocity_body_x = odom_est.twist.twist.linear.x;
      current_velocity_body_y = odom_est.twist.twist.linear.y;
    }

    double current_accel_body_x = cos(yaw_est) * accel_est.wrench.force.x +
                                  sin(yaw_est) * accel_est.wrench.force.y;

    /* don't really need these but might be informative */
    double current_accel_body_y = -sin(yaw_est) * accel_est.wrench.force.x +
                                  cos(yaw_est) * accel_est.wrench.force.y;

    saturate(desired_velocity, max_velocity_, -max_velocity_);

    /* If we have large target_heading_rel then start pointing else shoot */
    double de_vel = x_d.acceleration_or_force.x;  // - 0 * current_accel_body_x;
    if (fabs(e_yaw) > point_and_shoot_angle_) {
      desired_velocity = 0;
      de_vel = 0;
    }

    double e_vel = desired_velocity - current_velocity_body_x;

    /* Option 1: velocity controller via cascaded control */
    if (rc_has_throttle(rc) && in_offboard_ && fabs(e_vel) < max_vel_error_ &&
        fabs(velI.value) < max_velI_)
      velI.increment(e_vel, 1.0 / controller_freq_);
    double r_a_x = kp_vel_ * e_vel + kd_vel_ * de_vel + ki_vel_ * velI.value;
    // std::cout << "e_vel= " << e_vel;
    // std::cout << " kp_vel= " << kp_vel_;
    // std::cout << " ki_vel= " << ki_vel_;
    // std::cout << " velI.value= " << velI.value;
    // std::cout << " acc_ff= " << x_d.acceleration_or_force.x;

    // /* Option 2: non-cascaded position control */
    // double de_x = x_d.velocity.x - odom_est.twist.twist.linear.x;
    // double de_y = x_d.velocity.y - odom_est.twist.twist.linear.y;
    // double de_x_body = cos(yaw_est) * de_x + sin(yaw_est) * de_y;
    // if (fabs(posI.value) < max_posI_)
    //   posI.increment(e_x_body, 1.0 / controller_freq_);
    // double r_a_x =
    //     kp_pos_ * e_x_body + kd_pos_ * de_x_body +
    //     ki_pos_ * posI.value + x_d.acceleration_or_force.x;

    /* Yaw controller */
    if (rc_has_throttle(rc) && in_offboard_ && fabs(e_yaw) < max_yaw_error_ &&
        fabs(yawI.value) < max_yawI_)
      yawI.increment(e_yaw, 1.0 / controller_freq_);
    saturate(e_yaw, max_yaw_error_, -max_yaw_error_);
    saturate(de_yaw, max_yaw_rate_error_, -max_yaw_rate_error_);

    double r_M_z = kp_yaw_ * e_yaw + ki_yaw_ * yawI.value + kd_yaw_ * de_yaw;

    /* Convert desired forward accel to a pitch angle */
    double accel_desired =
        r_a_x / thrust_max_ * mass_;  // TODO: IS THIS CORRECT???
    // std::cout << " accel_d= " << accel_desired << std::endl;
    saturate(accel_desired, 1, -1);
    double pitch_desired = asin(accel_desired);
    if (pitch_testing_mode_) {
      ROS_WARN(
          "Pitch Testing Mode Enabled. Reading x_d.position.x as pitch in "
          "radians!");
      pitch_desired = x_d.position.x;  // for TESTING purposes
    }
    saturate(pitch_desired, max_pitch_, -max_pitch_);
    pitch_desired += pitch_offset_;

    /* Adjust thrust based on current pitch angle and desired accel */
    // double thrust_magnitude = mass_ * r_a_x / sin(pitch_est);
    // saturate(thrust_magnitude, thrust_max_, thrust_min_);
    double thrust_magnitude = thrust_max_;  // TODO: FOR TESTING

    /* Control desired pitch angle using feedback linearization.  To keep
    poles in left half plane, set k2_pitch_ < k1_pitch_ < 0. */
    /* Assuming the COM is below the axle and pitch=0 deg is stable. If
    COM is above the axle, make COM_length_ negative. */
    double e_pitch = dist_s1(pitch_desired, pitch_est);
    double de_pitch = -pitch_rate_est;

    if (rc_has_throttle(rc) && in_offboard_ &&
        fabs(e_pitch) < max_pitch_error_ && fabs(pitchI.value) < max_pitchI_)
      pitchI.increment(e_pitch, 1.0 / controller_freq_);
    saturate(e_pitch, max_pitch_error_, -max_pitch_error_);
    saturate(de_pitch, max_pitch_rate_error_, -max_pitch_rate_error_);

    double r_M_y =
        -COM_length_ / 9.81 * (-1.0 * sin(pitch_est)) / controller_freq_ +
        kp_pitch_ * e_pitch + ki_pitch_ * pitchI.value + kd_pitch_ * de_pitch;

    /* No desired roll moment */
    double r_M_x = 0;

    /* rotate desired moment from world frame to body frame */
    tf::Quaternion q_roll(0, 0, 0, 0), q_pitch(0, 0, 0, 0), q(0, 0, 0, 0);
    q_roll.setW(cos(roll_est / 2));  // TODO:  consider roll??
    q_roll.setX(sin(roll_est / 2));
    q_pitch.setW(cos(pitch_est / 2));
    q_pitch.setY(sin(pitch_est / 2));

    /* Compose Quaternions */
    q = q_pitch * q_roll;  // TODO: consider roll?
    tf::Matrix3x3 R(q);
    tf::Vector3 r_M(r_M_x, r_M_y, r_M_z);
    tf::Vector3 b_M = R * r_M;

    actuator_control.group_mix =
        mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;

    double mom_x = b_M.x() / (moment_roll_scale_ * max_thrust_rating_);
    double mom_y = b_M.y() / (moment_pitch_scale_ * max_thrust_rating_);
    double mom_z = b_M.z() / (moment_yaw_scale_ * max_thrust_rating_);

    saturate(mom_x, 1, -1);
    saturate(mom_y, 1, -1);
    saturate(mom_z, 1, -1);

    /* check if actuator commands are nan */
    if (mom_x != mom_x || mom_y != mom_y || mom_z != mom_z ||
        thrust_magnitude != thrust_magnitude) {
      mom_x = 0;
      mom_y = 0;
      mom_z = 0;
      thrust_magnitude = 0;
      ROS_ERROR("Got NaNs in actuator control message!");
    }

    actuator_control.controls[0] = mom_x;  // roll
    actuator_control.controls[1] =
        -mom_y;  // pitch //need minus sign to align to PX4 axes convention
    actuator_control.controls[2] = -mom_z;            // yaw
    actuator_control.controls[3] = thrust_magnitude;  // throttle

    /* build debug data */
    debug_data.target = x_d;
    debug_data.odom = odom_est;
    debug_data.vel_est_encoders = vel_est_encoders;
    debug_data.accel = accel_est;
    debug_data.rcout = rc;
    debug_data.actuator_control = actuator_control;

    debug_data.vel_body.x = current_velocity_body_x;
    debug_data.vel_body.y = current_velocity_body_y;
    debug_data.accel_body.x = current_accel_body_x;
    debug_data.accel_body.y = current_accel_body_y;
    debug_data.roll_desired = 0;
    debug_data.pitch_desired = pitch_desired;
    debug_data.yaw_desired = x_d.yaw;
    debug_data.r_M_y = r_M_y;
    debug_data.r_M_z = r_M_z;
    debug_data.r_M_x = r_M_x;
    debug_data.roll = roll_est;
    debug_data.pitch = pitch_est;
    debug_data.yaw = yaw_est;
    debug_data.roll_rate = roll_rate_est;
    debug_data.pitch_rate = pitch_rate_est;
    debug_data.yaw_rate = yaw_rate_est;
    debug_data.desired_velocity = desired_velocity;
    debug_data.posI = posI.value;
    debug_data.velI = velI.value;
    debug_data.yawI = yawI.value;
    debug_data.pitchI = pitchI.value;
    debug_data.throttle = thrust_magnitude;

    /* Controller Params */
    params_data.mass = mass_;
    params_data.rolling_thrust_max = thrust_max_;
    params_data.rolling_thrust_min = thrust_min_;
    params_data.rolling_kp_pos = kp_pos_;
    params_data.rolling_kp_pos = kd_pos_;
    params_data.rolling_kp_pos = ki_pos_;
    params_data.rolling_kp_vel = kp_vel_;
    params_data.rolling_ki_vel = ki_vel_;
    params_data.rolling_kp_yaw = kp_yaw_;
    params_data.rolling_ki_yaw = ki_yaw_;
    params_data.rolling_kd_yaw = kd_yaw_;
    params_data.rolling_max_vel_error = max_vel_error_;
    params_data.rolling_max_yaw_error = max_yaw_error_;
    params_data.rolling_max_pitch_error = max_pitch_error_;
    params_data.rolling_max_velI = max_velI_;
    params_data.rolling_max_yawI = max_yawI_;
    params_data.rolling_max_pitchI = max_pitchI_;
    params_data.rolling_max_pitch = max_pitch_;
    params_data.rolling_COM_length = COM_length_;
    params_data.rolling_kp_pitch = kp_pitch_;
    params_data.rolling_kd_pitch = kd_pitch_;
    params_data.rolling_ki_pitch = ki_pitch_;
    params_data.rolling_moment_roll_scale = moment_roll_scale_;
    params_data.rolling_moment_pitch_scale = moment_pitch_scale_;
    params_data.rolling_moment_yaw_scale = moment_yaw_scale_;
    params_data.rolling_max_velocity = max_velocity_;
    params_data.rolling_pitch_offset = pitch_offset_;
    params_data.rolling_point_and_shoot_angle = point_and_shoot_angle_;
    params_data.using_encoders = using_encoders_;

    // TODO: Add Testing mode information to debug/info
    std::cout << std::setprecision(4) << std::fixed << "\n\nROLLING MODE: "
              << "\nyaw: " << yaw_est << "\npitch: " << pitch_est
              << "\nroll: " << roll_est << "\nyaw_d: " << yaw_des
              << "\npitch_d: " << pitch_desired
              << "\nvel_d: " << desired_velocity
              << "\nvel_bx: " << current_velocity_body_x << "\nmom_x: " << mom_x
              << "\nmom_y: " << mom_y << "\nmom_z: " << mom_z
              << "\nthrust: " << thrust_magnitude
              << "\nhas_rc: " << rc_has_throttle(rc)
              << "\nin offboard: " << in_offboard_ << std::endl;
    return true;
  }

 private:
  Integrator posI, velI, yawI, pitchI;
  double yaw_deadzone_target;
  bool in_deadzone;
};
