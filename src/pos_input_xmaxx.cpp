#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <stdexcept>
#include "mobility_msgs/PositionTargetMode.h"
#include "ros/ros.h"

double time_start_ = 0.0;
bool trans_to_offboard_ = false;

void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
  std::string mode = msg->mode;
  if (mode == "OFFBOARD" && trans_to_offboard_ == false) {
    trans_to_offboard_ = true;
    time_start_ = ros::Time::now().toSec();
  } else if (mode != "OFFBOARD") {
    trans_to_offboard_ = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_xmaxx_pos_step_input");
  ros::NodeHandle nh;
  ros::Subscriber mode_sub = nh.subscribe("mavros/state", 10, stateCallback);
  ros::Publisher local_pos_pub =
      nh.advertise<mobility_msgs::PositionTargetMode>(
          "command/setpoint_raw/local", 10);
  std::string namespace_ = ros::this_node::getNamespace();
  namespace_.erase(0, 2);

  // Initialize config params
  double time_step;  // time to wait before step input is given
  double freq;       // freq. at which msgs will be published
  std::vector<double> pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, yaw, mode;
  std::vector<bool> vel_mode_x, vel_mode_y, vel_mode_z, ground_frame_z;

  /* Get params */
  ros::NodeHandle pnh("~");
  pnh.param<double>("time_step", time_step, 7.0);
  pnh.param<double>("freq", freq, 100);
  pnh.param<std::vector<double>>("pos_x", pos_x, {0});
  pnh.param<std::vector<double>>("pos_y", pos_y, {0});
  pnh.param<std::vector<double>>("pos_z", pos_z, {0});
  pnh.param<std::vector<double>>("vel_x", vel_x, {0});
  pnh.param<std::vector<double>>("vel_y", vel_y, {0});
  pnh.param<std::vector<double>>("vel_z", vel_z, {0});
  pnh.param<std::vector<double>>("yaw", yaw, {0});
  pnh.param<std::vector<double>>("mode", mode, {0});
  pnh.param<std::vector<bool>>("vel_mode_x", vel_mode_x, {0});
  pnh.param<std::vector<bool>>("vel_mode_y", vel_mode_y, {0});
  pnh.param<std::vector<bool>>("vel_mode_z", vel_mode_z, {0});
  pnh.param<std::vector<bool>>("ground_frame_z", ground_frame_z, {0});

  if (pos_x.size() != pos_y.size() || pos_y.size() != pos_z.size()) {
    std::cout << pos_x.size() << pos_y.size() << pos_z.size() << std::endl;
    ROS_ERROR("Waypoint size mismatch");
  }

  if (pos_x.size() < 1) ROS_ERROR("Less than 1 waypoint given");

  mobility_msgs::PositionTargetMode pos_sp;

  pos_sp.yaw = 0.0;
  pos_sp.yaw_rate = 0.0;
  pos_sp.mode = 0;  // 0 rolling, 1 flying
  ros::Rate loop_rate(freq);
  int i = 0;

  while (ros::ok()) {
    ros::spinOnce();
    double time_diff = ros::Time::now().toSec() - time_start_;
    pos_sp.type_mask = 0;
    if (trans_to_offboard_ == false || time_diff < time_step ||
        pos_x.size() == 1) {
      i = 0;
    } else {
      /* Get index of current waypoint based on time passed */
      i = floor(time_diff / time_step);
      i = (i >= pos_x.size()) ? pos_x.size() - 1 : i;
    }
    pos_sp.position.x = pos_x[i];
    pos_sp.position.y = pos_y[i];
    pos_sp.position.z = pos_z[i];
    pos_sp.yaw = yaw[i];
    pos_sp.mode = mode[i];
    pos_sp.velocity.x = vel_x[i];
    pos_sp.velocity.y = vel_y[i];
    pos_sp.velocity.z = vel_z[i];
    if (vel_mode_x[i])
      pos_sp.type_mask |= mobility_msgs::PositionTargetMode::IGNORE_PX;
    if (vel_mode_y[i])
      pos_sp.type_mask |= mobility_msgs::PositionTargetMode::IGNORE_PY;
    if (vel_mode_z[i])
      pos_sp.type_mask |= mobility_msgs::PositionTargetMode::IGNORE_PZ;
    if (ground_frame_z[i]) pos_sp.frame_id_z = namespace_ + "/ground";
    pos_sp.header.stamp = ros::Time::now();
    local_pos_pub.publish(pos_sp);
    loop_rate.sleep();
  }
}
