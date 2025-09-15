#!/bin/bash
source /ros_ws/install/setup.bash
source tb4_env.sh
ros2 launch moveit_control redis_receive_launch.py

