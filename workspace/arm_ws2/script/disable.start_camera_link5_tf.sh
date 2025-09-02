#/bin/bash
source tb4_env.sh
ros2 run tf2_ros static_transform_publisher --x -0.0201 --y 0.05067 --z -0.047272 --roll 3.1415926 --pitch 0.0 --yaw 0.0 --frame-id "link5" --child-frame-id "camera"