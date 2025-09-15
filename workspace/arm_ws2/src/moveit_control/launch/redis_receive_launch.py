from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


 # Declare arguments for transform parameters
# ARGUMENTS = [
#     DeclareLaunchArgument('x', default_value='0', description='X position of the transform'),
#     DeclareLaunchArgument('y', default_value='0', description='Y position of the transform'),
#     DeclareLaunchArgument('z', default_value='0', description='Z position of the transform'),
#     DeclareLaunchArgument('roll', default_value='0', description='Roll angle of the transform'),
#     DeclareLaunchArgument('pitch', default_value='0', description='Pitch angle of the transform'),
#     DeclareLaunchArgument('yaw', default_value='0', description='Yaw angle of the transform'),
#     DeclareLaunchArgument('robot_frame', default_value='base_link', description='Parent frame ID'),
#     DeclareLaunchArgument('laser_frame', default_value='laser', description='Child frame ID')
# ]
    
ARGUMENTS = []

def generate_launch_description():
    # TF static transform publisher node
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',              "-0.0201",
            '--y',              "0.05067",
            '--z',              "-0.047272",
            '--roll',           "3.1415926",
            '--pitch',          "0.0",
            '--yaw',            "0.0",
            '--frame-id',       "link5",
            '--child-frame-id', "camera"
        ],
        output='screen'
    )

    # Redis node
    redis_node = Node(
        package='moveit_control',
        executable='redis_receive',
        output='screen'
    )

    moveit_config = (
        MoveItConfigsBuilder("small_arm" , package_name="small_arm_moveit_config")
        #.robot_description(file_path="config/small_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/small_arm.srdf")
        #.trajectory_execution(file_path="config/moveit_controllers.yaml")
        #.planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )


    # Moveit node
    moveit_node = Node(
        package='moveit_control',
        executable='main',
        output='screen',
        #parameters=[
        #    moveit_config.to_dict()
        #]
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(tf_node)
    ld.add_action(redis_node)
    ld.add_action(moveit_node)
    return ld
