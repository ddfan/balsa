#include <ackermann_msgs/AckermannDriveStamped.h>
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

#include <tf/transform_listener.h>
#include <controller_xmaxx/PidConfig.h>

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
  double controller_freq_ = 50;
  const double pi_ = 3.1415962;
  
  bool first_ = true;
  tf::StampedTransform T_Imu_Blf_;
 public:
  PositionController() { load_params_common(); }

  virtual bool get_control_input(
      const ackermann_msgs::AckermannDriveStamped& x_d,
      const nav_msgs::Odometry& odom_est,
      const geometry_msgs::TwistStamped& vel_est_encoders,
      const geometry_msgs::WrenchStamped& accel_est,
      ackermann_msgs::AckermannDriveStamped& output,
      controller_xmaxx::DebugData& debug_data,
      controller_xmaxx::ParamsData& params_data) = 0;

  void load_params_common() {
    /* Read ros params, if not available then set to initialized value */
    ros::NodeHandle nhp("~");
    nhp.param<double>("controller_freq", controller_freq_, controller_freq_);
  }

 protected:
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
class AckermannController : public PositionController {
  double controller_freq_ = 50, kp_vel_ = 1.0, kd_vel_ = 0.5, ki_vel_ = 0.0,
         max_vel_error_ = 5.0, max_velI_ = 10.0, max_velocity_ = 0.1;

  bool using_encoders_ = false;

  bool lol = false; 

  


 public:
  tf::TransformListener listener_ss;
  AckermannController() : PositionController() {
    load_params();
    velI = Integrator();
    ROS_INFO("PID Controller Created!");

  }

  void change_pid_gains(controller_xmaxx::PidConfig &config, uint32_t level){
    ROS_INFO("Changing params");
    kp_vel_ = config.rolling_kp_vel;
    kd_vel_ = config.rolling_kd_vel;
    ki_vel_ = config.rolling_ki_vel;
  }

  void load_params() {
    /* Read ros params, if not available then set to initialized value */
    ros::NodeHandle nhp("~");
    nhp.param<double>("rolling_kp_vel", kp_vel_, kp_vel_);
    nhp.param<double>("rolling_kd_vel", kd_vel_, kd_vel_);
    nhp.param<double>("rolling_ki_vel", ki_vel_, ki_vel_);
    nhp.param<double>("controller_freq", controller_freq_, controller_freq_);
    nhp.param<double>("rolling_max_vel_error", max_vel_error_, max_vel_error_);
    nhp.param<double>("rolling_max_velI", max_velI_, max_velI_);

    nhp.param<double>("rolling_max_velocity", max_velocity_, max_velocity_);
    nhp.param<bool>("using_encoders", using_encoders_, using_encoders_);
  }

  bool get_control_input(const ackermann_msgs::AckermannDriveStamped& x_d,
                         const nav_msgs::Odometry& odom_est,
                         const geometry_msgs::TwistStamped& vel_est_encoders,
                         const geometry_msgs::WrenchStamped& accel_est,
                         ackermann_msgs::AckermannDriveStamped& output,
                         controller_xmaxx::DebugData& debug_data,
                         controller_xmaxx::ParamsData& params_data) {
    //###################################################################
    if(first_)
    {
      tf::TransformListener listener;
      
      try{
        listener.lookupTransform("/imu", "/base_link", ros::Time(0), T_Imu_Blf_); // ToDo: Get parent frame from odometry msg.
        first_ = false;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

    }

    tf::Quaternion rotquat_ImuBlf = T_Imu_Blf_.getRotation();
    tf::Matrix3x3 rotmat_ImuBLf;
    rotmat_ImuBLf.setRotation(rotquat_ImuBlf);

    // Transpose matrix.
    tf::Matrix3x3 rotmat_ImuBLf_trp = rotmat_ImuBLf.transpose();

    // Create vector 3 with tdotx, tdoty, tdotz.
    tf::Vector3 velocity_map_frame = tf::Vector3(odom_est.twist.twist.linear.x,
      odom_est.twist.twist.linear.y,
      odom_est.twist.twist.linear.z );
    tf::Vector3 velocity_body_frame = tf::Vector3(rotmat_ImuBLf_trp.tdotx(velocity_map_frame),
      rotmat_ImuBLf_trp.tdoty(velocity_map_frame),
      rotmat_ImuBLf_trp.tdotz(velocity_map_frame));

    //###################################################################

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

    double desired_velocity = x_d.drive.speed;

    /* Expect x_d.position and x_d.velocity in the world frame.  need to
       convert to
        body/rolling frame based on current yaw estimate */

    double current_velocity_body_x, current_velocity_body_y;
    if (using_encoders_) {
      current_velocity_body_x = vel_est_encoders.twist.linear.x;
      current_velocity_body_y = vel_est_encoders.twist.linear.y;
    } else {
      // ToDo
      current_velocity_body_x = velocity_body_frame.x();//odom_est.twist.twist.linear.x;
      current_velocity_body_y = velocity_body_frame.y();//odom_est.twist.twist.linear.y;
    }

    // double current_accel_body_x = cos(yaw_est) * accel_est.wrench.force.x +
    //                              sin(yaw_est) * accel_est.wrench.force.y;

    /* don't really need these but might be informative */
    // double current_accel_body_y = -sin(yaw_est) * accel_est.wrench.force.x +
    //                              cos(yaw_est) * accel_est.wrench.force.y;

    saturate(desired_velocity, max_velocity_, -max_velocity_);

    double e_vel = desired_velocity - current_velocity_body_x;

    /* velocity controller via cascaded control */
    if (fabs(e_vel) < max_vel_error_ && fabs(velI.value) < max_velI_)
      velI.increment(e_vel, 1.0 / controller_freq_);
    double r_a_x = kp_vel_ * e_vel + ki_vel_ * velI.value;  // kd_vel_ * de_vel;
    // std::cout << "e_vel= " << e_vel;
    // std::cout << " kp_vel= " << kp_vel_;
    // std::cout << " ki_vel= " << ki_vel_;
    // std::cout << " velI.value= " << velI.value;
    // std::cout << " acc_ff= " << x_d.acceleration_or_force.x;

    /* create message */
    output = x_d;
    double speed_output = x_d.drive.speed + r_a_x;
    saturate(speed_output, max_velocity_, -max_velocity_);
    output.drive.speed = x_d.drive.speed + r_a_x;

    /* check if actuator commands are nan */
    if (output.drive.speed != output.drive.speed) {
      output.drive.speed = 0;
      ROS_ERROR("Got NaNs in actuator control message!");
    }

    /* build debug data */
    debug_data.target = x_d;
    debug_data.odom = odom_est;
    debug_data.vel_est_encoders = vel_est_encoders;
    debug_data.accel = accel_est;
    debug_data.output = output;

    debug_data.vel_body.x = current_velocity_body_x;
    debug_data.vel_body.y = current_velocity_body_y;
    //debug_data.accel_body.x = current_accel_body_x;
    //debug_data.accel_body.y = current_accel_body_y;
    debug_data.roll = roll_est;
    debug_data.pitch = pitch_est;
    debug_data.yaw = yaw_est;
    debug_data.roll_rate = roll_rate_est;
    debug_data.pitch_rate = pitch_rate_est;
    debug_data.yaw_rate = yaw_rate_est;
    debug_data.desired_velocity = desired_velocity;
    debug_data.velI = velI.value;

    /* Controller Params */
    params_data.rolling_kp_vel = kp_vel_;
    params_data.rolling_ki_vel = ki_vel_;
    params_data.rolling_max_vel_error = max_vel_error_;
    params_data.rolling_max_velI = max_velI_;
    params_data.rolling_max_velocity = max_velocity_;
    params_data.using_encoders = using_encoders_;

    // TODO: Add Testing mode information to debug/info
    /*std::cout << std::setprecision(4) << std::fixed
              << "\n\n CONTROLLER_XMAXX DEBUG: "
              << "\nyaw: " << yaw_est << "\npitch: " << pitch_est
              << "\nroll: " << roll_est << "\nvel_d: " << desired_velocity
              << "\nvel_bx: " << current_velocity_body_x
              << "\nsteering_angle: " << output.drive.steering_angle;
    */
    return true;
  }

 private:
  Integrator velI;
};
