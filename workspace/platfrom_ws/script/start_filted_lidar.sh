#!/bin/bash
source tb4_env.sh
ros2 launch lidar_filter filted_lidar_launch.py x:=-0.005 y:=0 z:=0 roll:=3.1415926 pitch:=0 yaw:=4.7123886 robot-frame:=base_link laser_frame:=laser
