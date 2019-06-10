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
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/RCOut.h"
#include "mobility_msgs/PositionTargetMode.h"

enum class ControlInputType { ATTITUDE_TARGET, ACTUATOR_CONTROL };
typedef mobility_msgs::PositionTargetMode Goal;

struct ControlInputMessage {
  ControlInputType control_input_type;
  mavros_msgs::AttitudeTarget attitude_target;
  mavros_msgs::ActuatorControl actuator_control;
};

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
  double mass_ = 2.0, flying_max_thrust_ = 0.75, flying_min_thrust_ = 0.45,
         max_thrust_rating_ = 60.0, controller_freq_ = 50;
  const double pi_ = 3.1415962;
  bool in_offboard_ = false;

 public:
  PositionController() { load_params_common(); }

  virtual bool get_control_input(
      const mobility_msgs::PositionTargetMode& x_d,
      const mavros_msgs::RCOut& rc, const nav_msgs::Odometry& odom_est,
      const geometry_msgs::TwistStamped& vel_est_encoders,
      const geometry_msgs::WrenchStamped& accel_est, const double clearance_,
      ControlInputMessage& control_input_message,
      controller_xmaxx::DebugData& debug_data,
      controller_xmaxx::ParamsData& params_data) = 0;

  void setOffboard(const bool in_offboard) { in_offboard_ = in_offboard; }

  void load_params_common() {
    /* Read ros params, if not available then set to initialized value */
    ros::NodeHandle nhp("~");
    nhp.param<double>("mass", mass_, mass_);
    nhp.param<double>("flying_max_thrust", flying_max_thrust_,
                      flying_max_thrust_);
    nhp.param<double>("flying_min_thrust", flying_min_thrust_,
                      flying_min_thrust_);
    nhp.param<double>("max_thrust_rating", max_thrust_rating_,
                      max_thrust_rating_);
    nhp.param<double>("controller_freq", controller_freq_, controller_freq_);
  }

 protected:
  mavros_msgs::AttitudeTarget get_attitude_target() {
    // check we have the correct type
    if (control_input_type_ == ControlInputType::ATTITUDE_TARGET)
      return attitude_target_;
    else
      ROS_ERROR("Cannot get target attitude in actuator control mode.");
  }

  mavros_msgs::ActuatorControl get_actuator_control() {
    // check we have the correct type
    if (control_input_type_ == ControlInputType::ACTUATOR_CONTROL)
      return actuator_control_;
    else
      ROS_ERROR("Cannot get actuator control mode in attitude target mode.");
  }

  ControlInputMessage update_attitude_target(
      mavros_msgs::AttitudeTarget& attitude_target) {
    attitude_target_ = attitude_target;
    control_input_type_ = ControlInputType::ATTITUDE_TARGET;

    ControlInputMessage message;
    message.control_input_type = control_input_type_;
    message.attitude_target = attitude_target_;
    message.actuator_control = actuator_control_;
    return message;
  }

  ControlInputMessage update_actuator_control(
      mavros_msgs::ActuatorControl& actuator_control) {
    actuator_control_ = actuator_control;
    control_input_type_ = ControlInputType::ACTUATOR_CONTROL;

    ControlInputMessage message;
    message.control_input_type = control_input_type_;
    message.attitude_target = attitude_target_;
    message.actuator_control = actuator_control_;
    return message;
  }

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

 private:
  mavros_msgs::AttitudeTarget attitude_target_;
  mavros_msgs::ActuatorControl actuator_control_;
  ControlInputType control_input_type_;
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
                         const double clearance_,
                         ControlInputMessage& control_input_message,
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
    mavros_msgs::ActuatorControl actuator_control;

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

    control_input_message = update_actuator_control(actuator_control);

    /* build debug data */
    debug_data.target = x_d;
    debug_data.odom = odom_est;
    debug_data.vel_est_encoders = vel_est_encoders;
    debug_data.accel = accel_est;
    debug_data.rcout = rc;
    debug_data.actuator_control = control_input_message.actuator_control;
    debug_data.clearance = clearance_;

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

class FlyingControllerBasic : public PositionController {
  double kp_x_ = 1, kp_y_ = 1, kp_z_ = 1, kd_x_ = 1, kd_y_ = 1, kd_z_ = 1,
         ki_x_ = 0, ki_y_ = 0, ki_z_ = 0, max_x_error_to_integrate_ = 1,
         max_y_error_to_integrate_ = 1, max_z_error_to_integrate_ = 1,
         max_xI_ = 0, max_yI_ = 0, max_zI_ = 0, max_e_pos_ = 1, max_e_vel_ = 1,
         max_acc_xy_ = 1, max_acc_z_ = 10.81, min_acc_z_ = 8.81;
  bool cascaded_flight_control_ = true;

 public:
  FlyingControllerBasic() : PositionController() {
    load_params();
    xI = Integrator();
    yI = Integrator();
    zI = Integrator();
  }

  void load_params() {
    /* Read ros params, if not available then set to initialized value */
    ros::NodeHandle nhp("~");
    nhp.param<double>("flying_kp_x", kp_x_, kp_x_);
    nhp.param<double>("flying_kp_y", kp_y_, kp_y_);
    nhp.param<double>("flying_kp_z", kp_z_, kp_z_);
    nhp.param<double>("flying_kd_x", kd_x_, kd_x_);
    nhp.param<double>("flying_kd_y", kd_y_, kd_y_);
    nhp.param<double>("flying_kd_z", kd_z_, kd_z_);
    nhp.param<double>("flying_ki_x", ki_x_, ki_x_);
    nhp.param<double>("flying_ki_y", ki_y_, ki_y_);
    nhp.param<double>("flying_ki_z", ki_z_, ki_z_);
    nhp.param<double>("flying_max_x_error_to_integrate",
                      max_x_error_to_integrate_, max_x_error_to_integrate_);
    nhp.param<double>("flying_max_y_error_to_integrate",
                      max_y_error_to_integrate_, max_y_error_to_integrate_);
    nhp.param<double>("flying_max_z_error_to_integrate",
                      max_z_error_to_integrate_, max_z_error_to_integrate_);
    nhp.param<double>("flying_max_xI", max_xI_, max_xI_);
    nhp.param<double>("flying_max_yI", max_yI_, max_yI_);
    nhp.param<double>("flying_max_zI", max_zI_, max_zI_);
    nhp.param<double>("flying_max_e_pos", max_e_pos_, max_e_pos_);
    nhp.param<double>("flying_max_e_vel", max_e_vel_, max_e_vel_);
    nhp.param<double>("flying_max_acc_xy", max_acc_xy_, max_acc_xy_);
    nhp.param<double>("flying_max_acc_z", max_acc_z_, max_acc_z_);
    nhp.param<double>("flying_min_acc_z", min_acc_z_, min_acc_z_);

    nhp.param<bool>("cascaded_flight_control", cascaded_flight_control_,
                    cascaded_flight_control_);
  }

  bool get_control_input(const mobility_msgs::PositionTargetMode& x_d,
                         const mavros_msgs::RCOut& rc,
                         const nav_msgs::Odometry& odom_est,
                         const geometry_msgs::TwistStamped& vel_est_encoders,
                         const geometry_msgs::WrenchStamped& accel_est,
                         double clearance_,
                         ControlInputMessage& control_input_message,
                         controller_xmaxx::DebugData& debug_data,
                         controller_xmaxx::ParamsData& params_data) {
    double z_pos;
    if (clearance_ != clearance_)
      z_pos = odom_est.pose.pose.position.z;
    else
      z_pos = clearance_;

    /* Calculate error and saturate for sanity */
    double e_x = 0, e_y = 0, e_z = 0;
    /* Update error in position if position is not ignored */
    if ((x_d.type_mask & Goal::IGNORE_PX) != Goal::IGNORE_PX)
      e_x = x_d.position.x - odom_est.pose.pose.position.x;
    if ((x_d.type_mask & Goal::IGNORE_PY) != Goal::IGNORE_PY)
      e_y = x_d.position.y - odom_est.pose.pose.position.y;
    if ((x_d.type_mask & Goal::IGNORE_PZ) != Goal::IGNORE_PZ) {
      e_z = x_d.position.z - z_pos;
    }
    saturate(e_x, max_e_pos_, -max_e_pos_);
    saturate(e_y, max_e_pos_, -max_e_pos_);
    saturate(e_z, max_e_pos_, -max_e_pos_);

    double int_x = e_x, int_y = e_y, int_z = e_z;

    /* odom msg has velocity in base_link frame, transform to map */
    Eigen::Vector3d vel_est(odom_est.twist.twist.linear.x,
                            odom_est.twist.twist.linear.y,
                            odom_est.twist.twist.linear.z);
    Eigen::Quaterniond temp_q(
        odom_est.pose.pose.orientation.w, odom_est.pose.pose.orientation.x,
        odom_est.pose.pose.orientation.y, odom_est.pose.pose.orientation.z);
    Eigen::Matrix3d R = temp_q.toRotationMatrix();
    vel_est = R * vel_est;

    double de_x = 0, de_y = 0, de_z = 0;
    if ((x_d.type_mask & Goal::IGNORE_VX) != Goal::IGNORE_VX)
      de_x = x_d.velocity.x - vel_est(0);
    if ((x_d.type_mask & Goal::IGNORE_VY) != Goal::IGNORE_VY)
      de_y = x_d.velocity.y - vel_est(1);
    if ((x_d.type_mask & Goal::IGNORE_VZ) != Goal::IGNORE_VZ)
      de_z = x_d.velocity.z - vel_est(2);

    /* transform e_x, e_y and de_x, de_y by yaw to get base_link_g_aligned frame
     */
    double roll, pitch, yaw;
    get_rpy_from_pose(odom_est.pose.pose.orientation, roll, pitch, yaw);
    double e_x_body, e_y_body, de_x_body, de_y_body;
    e_x_body = cos(yaw) * e_x + sin(yaw) * e_y;
    e_y_body = -sin(yaw) * e_x + cos(yaw) * e_y;
    de_x_body = cos(yaw) * de_x + sin(yaw) * de_y;
    de_y_body = -sin(yaw) * de_x + cos(yaw) * de_y;

    if (cascaded_flight_control_) {
      de_x_body += e_x_body * kp_x_;
      de_y_body += e_y_body * kp_y_;
      de_z += e_z * kp_z_;

      e_x_body = 0;
      e_y_body = 0;
      e_z = 0;

      int_x = de_x_body;
      int_y = de_y_body;
      int_z = de_z;
    }

    saturate(de_x_body, max_e_vel_, -max_e_vel_);
    saturate(de_y_body, max_e_vel_, -max_e_vel_);
    saturate(de_z, max_e_vel_, -max_e_vel_);

    if (rc_has_throttle(rc) && in_offboard_ &&
        fabs(int_x) < max_x_error_to_integrate_ &&
        fabs(xI.value) * ki_x_ < max_xI_)
      xI.increment(int_x, 1.0 / controller_freq_);
    if (rc_has_throttle(rc) && in_offboard_ &&
        fabs(int_y) < max_y_error_to_integrate_ &&
        fabs(yI.value) * ki_y_ < max_yI_)
      yI.increment(int_y, 1.0 / controller_freq_);
    if (rc_has_throttle(rc) && in_offboard_ &&
        fabs(int_z) < max_z_error_to_integrate_ &&
        fabs(zI.value) * ki_z_ < max_zI_)
      zI.increment(int_z, 1.0 / controller_freq_);

    double des_accel_x_body =
        kp_x_ * e_x_body + kd_x_ * de_x_body + ki_x_ * xI.value;
    double des_accel_y_body =
        kp_y_ * e_y_body + kd_y_ * de_y_body + ki_y_ * yI.value;
    double des_accel_z = kp_z_ * e_z + kd_z_ * de_z + ki_z_ * zI.value;

    /* convert desired accelerations back to map frame */
    double des_accel_x =
        cos(yaw) * des_accel_x_body - sin(yaw) * des_accel_y_body;
    double des_accel_y =
        sin(yaw) * des_accel_x_body + cos(yaw) * des_accel_y_body;

    /* PID Control Law with Feedfoward Acceleration */
    Eigen::Vector3d thrust;
    thrust(0) = mass_ * (des_accel_x + x_d.acceleration_or_force.x);
    thrust(1) = mass_ * (des_accel_y + x_d.acceleration_or_force.y);
    thrust(2) = mass_ * (des_accel_z + x_d.acceleration_or_force.z + 9.81);
    thrust(0) =
        mass_ * saturate_by_value(thrust(0) / mass_, max_acc_xy_, -max_acc_xy_);
    thrust(1) =
        mass_ * saturate_by_value(thrust(1) / mass_, max_acc_xy_, -max_acc_xy_);
    thrust(2) =
        mass_ * saturate_by_value(thrust(2) / mass_, max_acc_z_, -min_acc_z_);
    double thrust_mag = thrust.norm();

    /* Generate target attitude from desired thrust */
    // BASED ON TAEYOUNG LEE's PAPER
    double yaw_des = x_d.yaw;
    Eigen::Vector3d heading_axis(1, 0, 0);
    Eigen::Matrix3d R_yaw(3, 3);
    R_yaw(0, 0) = cos(yaw_des);
    R_yaw(0, 1) = -sin(yaw_des);
    R_yaw(0, 2) = 0;
    R_yaw(1, 0) = sin(yaw_des);
    R_yaw(1, 1) = cos(yaw_des);
    R_yaw(1, 2) = 0;
    R_yaw(2, 0) = 0;
    R_yaw(2, 1) = 0;
    R_yaw(2, 2) = 1;

    Eigen::Quaternion<double> q_test(R_yaw);
    tf::Quaternion quat_test(q_test.x(), q_test.y(), q_test.z(), q_test.w());
    tf::Matrix3x3 m_test(quat_test);
    double roll_test, pitch_test, yaw_test;
    m_test.getRPY(roll_test, pitch_test, yaw_test);

    // std::cout << "yaw_t: " << yaw_test << "\tpitch_t: " << pitch_test
    //          << "\troll_t: " << roll_test << std::endl;

    Eigen::Vector3d des_heading_vect = R_yaw * heading_axis;
    Eigen::Vector3d b3_axis = thrust.normalized();
    Eigen::Vector3d b2_axis = b3_axis.cross(des_heading_vect);
    Eigen::Vector3d b1_axis = b2_axis.cross(b3_axis);

    Eigen::Matrix3d orientation(3, 3);
    orientation.col(0) = b1_axis;
    orientation.col(1) = b2_axis;
    orientation.col(2) = b3_axis;
    Eigen::Quaternion<double> q(orientation);

    mavros_msgs::AttitudeTarget att_des;
    att_des.orientation.x = q.x();
    att_des.orientation.y = q.y();
    att_des.orientation.z = q.z();
    att_des.orientation.w = q.w();
    att_des.thrust = thrust_mag / max_thrust_rating_;
    att_des.thrust = saturate_by_value(att_des.thrust, flying_max_thrust_,
                                       flying_min_thrust_);

    /* if ignoring pos and velocity both, make sure desired orientation is flat.
     * for use in flying idling on the ground.
     */
    if ((x_d.type_mask & Goal::IGNORE_PX) == Goal::IGNORE_PX &&
        (x_d.type_mask & Goal::IGNORE_PY) == Goal::IGNORE_PY &&
        (x_d.type_mask & Goal::IGNORE_PZ) == Goal::IGNORE_PZ &&
        (x_d.type_mask & Goal::IGNORE_VX) == Goal::IGNORE_VX &&
        (x_d.type_mask & Goal::IGNORE_VY) == Goal::IGNORE_VY &&
        (x_d.type_mask & Goal::IGNORE_VZ) == Goal::IGNORE_VZ) {
      tf::Quaternion q_yaw(0, 0, 0, 0);
      q_yaw.setW(cos(yaw_des / 2));
      q_yaw.setZ(sin(yaw_des / 2));

      att_des.orientation.x = q_yaw.x();
      att_des.orientation.y = q_yaw.y();
      att_des.orientation.z = q_yaw.z();
      att_des.orientation.w = q_yaw.w();
    }

    /* check if desired attitude commands are nan */
    if (att_des.orientation.x != att_des.orientation.x ||
        att_des.orientation.y != att_des.orientation.y ||
        att_des.orientation.z != att_des.orientation.z ||
        att_des.orientation.w != att_des.orientation.w ||
        att_des.thrust != att_des.thrust) {
      att_des.orientation.x = 0;  // TODO:  update these to be zero roll/pitch
      att_des.orientation.y = 0;
      att_des.orientation.z = 0;
      att_des.orientation.w = 1;
      att_des.thrust = mass_ * 9.81 / max_thrust_rating_;
      ROS_ERROR("Got NaNs in actuator control message!");
    }

    /* Get rpy angles from x_est quaternions */
    tf::Quaternion quat(
        odom_est.pose.pose.orientation.x, odom_est.pose.pose.orientation.y,
        odom_est.pose.pose.orientation.z, odom_est.pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    /* Get rpy angles from att_des quaternions */
    tf::Quaternion quat2(att_des.orientation.x, att_des.orientation.y,
                         att_des.orientation.z, att_des.orientation.w);
    tf::Matrix3x3 m2(quat2);
    double roll_desired, pitch_desired, yaw_desired;
    m2.getRPY(roll_desired, pitch_desired, yaw_desired);

    control_input_message = update_attitude_target(att_des);

    /* build debug data */
    debug_data.target = x_d;
    debug_data.odom = odom_est;
    debug_data.vel_est_encoders = vel_est_encoders;
    debug_data.accel = accel_est;
    debug_data.clearance = clearance_;
    debug_data.roll_desired = roll_desired;
    debug_data.pitch_desired = pitch_desired;
    debug_data.yaw_desired = yaw_desired;
    debug_data.roll = roll;
    debug_data.pitch = pitch;
    debug_data.yaw = yaw;
    debug_data.throttle = att_des.thrust;
    debug_data.attitude_target = control_input_message.attitude_target;

    debug_data.error_pos.x = e_x;
    debug_data.error_pos.y = e_y;
    debug_data.error_pos.z = e_z;

    debug_data.error_vel.x = de_x;
    debug_data.error_vel.y = de_y;
    debug_data.error_vel.z = de_z;

    debug_data.thrust.x = thrust(0) / mass_;
    debug_data.thrust.y = thrust(1) / mass_;
    debug_data.thrust.z = thrust(2) / mass_;

    debug_data.xI = xI.value;
    debug_data.yI = yI.value;
    debug_data.zI = zI.value;

    /* Controller Params */
    params_data.cascaded_flight_control = cascaded_flight_control_;
    params_data.flying_min_thrust = flying_min_thrust_;
    params_data.flying_max_thrust = flying_max_thrust_;
    params_data.mass = mass_;
    params_data.controller_freq = controller_freq_;
    params_data.flying_kp_x = kp_x_;
    params_data.flying_kp_y = kp_y_;
    params_data.flying_kp_z = kp_z_;
    params_data.flying_kd_x = kd_x_;
    params_data.flying_kd_y = kd_y_;
    params_data.flying_kd_z = kd_z_;
    params_data.flying_ki_x = ki_x_;
    params_data.flying_ki_y = ki_y_;
    params_data.flying_ki_z = ki_z_;
    params_data.flying_max_x_error_to_integrate = max_x_error_to_integrate_;
    params_data.flying_max_y_error_to_integrate = max_y_error_to_integrate_;
    params_data.flying_max_z_error_to_integrate = max_z_error_to_integrate_;
    params_data.flying_max_xI = max_xI_;
    params_data.flying_max_yI = max_yI_;
    params_data.flying_max_zI = max_zI_;

    std::cout << "\nFLYING MODE:"
              << "\nroll: " << roll << "\npitch: " << pitch << "\nyaw: " << yaw
              << "\nroll_des: " << roll_desired
              << "\npitch_des: " << pitch_desired
              << "\nyaw_des: " << yaw_desired << "\nthrust: " << att_des.thrust
              << std::endl;
    return true;
  }

 private:
  Integrator xI, yI, zI;
};
