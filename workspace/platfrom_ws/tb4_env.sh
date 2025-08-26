#!/bin/bash
# export CYCLONEDDS_URI=/tb4_dds.xml
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export ROS_DOMAIN_ID=17
# source ./install/setup.bash

# export ROS_DISCOVERY_SERVER=TCPv4:[192.168.1.203]:42100
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_SUPER_CLIENT=TRUE
# export FASTRTPS_DEFAULT_PROFILES_FILE=~/workspace/rtps.xml

ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

# fastdds discovery --server-id 0
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# export FASTRTPS_DEFAULT_PROFILES_FILE=/super_client_cfg.xml
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

fastdds discovery --server-id 0 -l 192.168.1.203 -p 11811 -l 192.168.186.3 -p 11811 -l 127.0.0.1 -p 11811
# fastdds discovery --server-id 0 -t 192.168.1.203 -q 42100 -l 192.168.186.3 -p 11811 -l 127.0.0.1 -p 11811