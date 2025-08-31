#/bin/bash
ros2 service call /arm_goal_pose python_moveit_interface/srv/PoseRequest "{target_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, message: 'home'}"
ros2 service call /arm_goal_pose python_moveit_interface/srv/PoseRequest "{target_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, message: 'detect'}"
ros2 service call /arm_goal_pose python_moveit_interface/srv/PoseRequest "{target_pose: {position: {x: 0.0, y: 0.16, z: 0.22}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, message: ''}"
ros2 service call /arm_goal_finish std_srvs/srv/Trigger