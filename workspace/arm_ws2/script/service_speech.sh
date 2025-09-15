#/bin/bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /speech_recognition std_msgs/msg/Int32 "{data: 2}"
