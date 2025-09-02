#/bin/bash
source tb4_env.sh
ros2 service call /goal_request movement_platform_if/srv/GoalRequest "{goal_pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"

# colcon build --packages-select=navigation
# ros2 service call /goal_status movement_platform_if/srv/GoalStatus
# ros2 run navigation nav_node --ros-args --remap /cmd_vel_test:=/cmd_vel -p max_linear_accel:=0.2 -p max_angular_accel:=0.2 -p kp_avoidance:=0.1