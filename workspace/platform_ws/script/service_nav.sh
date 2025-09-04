#/bin/bash
source tb4_env.sh
ros2 service call /goal_request movement_platform_if/srv/GoalRequest "{goal_pose: {position: {x: 0.6, y: -1.5, z: 0.0}, orientation: {w: 1.0}}}"
sleep 15
ros2 service call /goal_request movement_platform_if/srv/GoalRequest "{goal_pose: {position: {x: -0.03, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# ros2 service call /goal_request movement_platform_if/srv/GoalRequest "{goal_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
# colcon build --packages-select=navigation
# ros2 service call /goal_status movement_platform_if/srv/GoalStatus
# os2 run navigation nav_node --ros-args --remap /cmd_vel_test:=/cmd_vel -p max_linear_accel:=0.4 -p max_angular_accel:=0.8 -p kp_avoidance:=0.1 -p max_angular_vel:=0.8 -p max_linear_vel:=0.25 -p kp_linear:=1.5 -p kp_angular:=0.02 -p goal_dist_tolerance:=0.04 -p kp_avoidance:=0.005 -p avoidance_activation_dist:=0.8

# ros2 run navigation nav_node --ros-args  -p max_linear_accel:=0.4 -p max_angular_accel:=0.8 -p kp_avoidance:=0.1 -p max_angular_vel:=0.8 -p max_linear_vel:=0.25 -p kp_linear:=1.5 -p kp_angular:=0.2 -p goal_dist_tolerance:=0.04 -p kp_avoidance:=0.001 -p avoidance_activation_dist:=0.5
# ros2 run navigation nav_node --ros-args --remap /cmd_vel_test:=/cmd_vel -p max_linear_accel:=0.4 -p max_angular_accel:=0.8 -p kp_avoidance:=0.1 -p max_angular_vel:=0.8 -p max_linear_vel:=0.25 -p kp_linear:=1.5 -p kp_angular:=0.2 -p goal_dist_tolerance:=0.04 -p kp_avoidance:=0.001 -p avoidance_activation_dist:=0.5
# ros2 run navigation nav_node --ros-args --remap /cmd_vel_test:=/cmd_vel -p max_linear_accel:=0.4 -p max_angular_accel:=0.8 -p kp_avoidance:=0.1 -p max_angular_vel:=0.8 -p max_linear_vel:=0.25 -p kp_linear:=1.5 -p kp_angular:=0.9 -p goal_dist_tolerance:=0.04 -p kp_avoidance:=0.001 -p avoidance_activation_dist:=0.5

