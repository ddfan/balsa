#!/bin/bash
SESSION=pos_step_input

# Split Panes
tmux -2 new-session -d -s $SESSION
#tmux set pane-border-status top
tmux split-window -h  # split horizontally
tmux split-window -v 
tmux select-pane -t 0
tmux split-window -v  # split vertically
tmux split-window -h
tmux resize-pane -R 20
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 4
tmux split-window -h -p 33
tmux select-pane -t 6
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -h
tmux select-pane -t 7
tmux split-window -h
tmux select-pane -t 4
tmux split-window -h
# Exec Commands
tmux send-keys -t 0 "roscore" C-m
#tmux send-keys -t 1 "sleep 2; rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 0.0 world map 10" C-m
#tmux send-keys -t 2 "sleep 2; rosrun sensor_fakemocap sensor_mocap" C-m
#tmux send-keys -t 2 "sleep 2; roslaunch realsense2_camera rs_aligned_depth.launch" C-m

tmux send-keys -t 1 "sleep 1; roslaunch mavros_custom_launch px4.launch robot_namespace:=rollo" C-m
tmux send-keys -t 2 "sleep 1; roslaunch orb_convertor orb_convertor.launch &" C-m
tmux send-keys -t 4 "sleep 2; roslaunch ORB_SLAM2 ORB_SLAM2.launch" C-m
tmux send-keys -t 3 "sleep 2; roslaunch realsense2_camera 60FPS.launch" C-m
tmux send-keys -t 5 "sleep 1; roslaunch hw_interface hw_interface.launch" C-m
tmux send-keys -t 6 "sleep 1; roslaunch controller_hybrid controller_hybrid.launch" C-m
tmux send-keys -t 7 "sleep 1; roslaunch controller_hybrid pos_input.launch" C-m
tmux send-keys -t 8 "sleep 1; rostopic echo /rollo/mavros/state" C-m
tmux send-keys -t 9 "sleep 1; rostopic echo /rollo/mavros/local_position/pose" C-m
tmux send-keys -t 10 "sleep 1; rostopic echo /rollo/command/setpoint_raw/local" C-m
#tmux send-keys -t 9 "sleep 2; rosrun orb_convertor orb_convertor" C-m
#tmux send-keys -t 10 "sleep 3; rosrun ORB_SLAM2 RGBD /home/woody/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/woody/Downloads/RealSense.yaml /camera/color/image_raw /camera/aligned_depth_to_color/image_raw" C-m
tmux send-keys -t 11 "roscd flightstack_px4/scripts" C-m
tmux send-keys -t 11 "./reboot_px4.sh"
#tmux send-keys -t 11 "sleep 1; rostopic echo /rollo/mavros/vision_pose/pose" C-m
#tmux send-keys -t 12 "cd /home/woody/altair_bags" C-m
#tmux send-keys -t 12 "../bag_record.sh"
tmux send-keys -t 12 "sleep 1; roslaunch localizer_clearance localizer_clearance.launch" C-m
# tmux send-keys -t 0 "sleep 4; rostopic echo /mavros/local_position/pose" C-m
# tmux send-keys -t 1 "sleep 4; rostopic echo /quadrotor/setpoint_raw/pose" C-m
# tmux send-keys -t 2 "sleep 4; rostopic echo /mavros/setpoint_raw/target_attitude" C-m
# tmux send-keys -t 3 "sleep 4; rostopic echo /mavros/setpoint_raw/attitude " C-m
# tmux send-keys "sleep 2; roslaunch visualizer_basic visualizer_basic.launch" C-m

tmux select-pane -t 2
tmux split-window -h
tmux send-keys -t 3 "sleep 1; roslaunch encoder_filter encoder_filter.launch" C-m
#tmux select-pane -t 4
#tmux split-window -h
#tmux send-keys -t 5 "sleep 1; roslaunch velocity_transformation velocity_transformation.launch" C-m
tmux -2 attach-session -t $SESSION
