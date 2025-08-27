#!/bin/bash
source tb4_env.sh
# export ROS_DOMAIN_ID=70
# ros2 launch turtlebot4_navigation nav2.launch.py params_file:=/home/orin/iansun2/config/nav2.yaml
ros2 launch movement_platform nav_launch.py params_file:=/root/workspace/platform_ws/config/nav2_road.yaml map:=./map/robotsot0403_road.yaml

