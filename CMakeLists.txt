cmake_minimum_required(VERSION 2.8.3)
project(controller_adaptiveclbf)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rospy std_msgs actionlib_msgs)

add_message_files(
  FILES
  DebugData.msg
)

add_service_files(DIRECTORY srv FILES AddData2Model.srv PredictModel.srv)
add_action_files(DIRECTORY action FILES TrainModel.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/adaptiveClbf.cfg
  cfg/figureEight.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rospy std_msgs
)

install(PROGRAMS src/adaptive_clbf_node.py src/figure_eight.py src/encoder_odometry.py src/model_service.py src/dyn_reconfig_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
