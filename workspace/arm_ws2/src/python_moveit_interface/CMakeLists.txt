cmake_minimum_required(VERSION 3.8)
project(python_moveit_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PoseRequest.srv"
  "srv/GripperControl.srv"
  "srv/AutoPose.srv"
  "srv/ArmControl.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  std_msgs
)

ament_package()
