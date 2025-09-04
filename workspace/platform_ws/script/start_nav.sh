#!/bin/bash
source tb4_env.sh
# export ROS_DOMAIN_ID=70
# ros2 launch turtlebot4_navigation nav2.launch.py params_file:=/home/orin/iansun2/config/nav2.yaml
ros2 run navigation nav_node --ros-args --remap /cmd_vel_test:=/cmd_vel -p max_linear_accel:=0.4 -p max_angular_accel:=0.8 -p kp_avoidance:=0.1 \
    -p max_angular_vel:=0.8 -p max_linear_vel:=0.25 -p kp_linear:=1.5 -p kp_angular:=0.9 -p goal_dist_tolerance:=0.04 -p kp_avoidance:=0.001 -p avoidance_activation_dist:=0.5
# ros2 launch movement_platform nav_launch.py params_file:=/root/workspace/platform_ws/config/nav2_road.yaml map:=./map/robotsot0403_road.yaml
