#!/bin/bash

echo "### Cleaning the previous session ###"

rosclean purge -y

# Kill 
killall -9 screen

# Kill gazebo session
killall -9 gzclient
killall -9 gzserver

# Kill ROS master
killall -9 rosmaster
killall -9 rosout

killall -9 default_cfg
killall -9 roslaunch
killall -9 master_discovery
killall -9 master_sync

killall -9 python
killall -9 robot_state_publisher

screen -wipe

# Show status
echo ""
echo "### Remaining ROS processes ###"
ps aux | grep ros | grep -v "grep"

exit 0

