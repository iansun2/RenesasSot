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


 # Declare arguments for transform parameters
ARGUMENTS = [
    DeclareLaunchArgument('x', default_value='0', description='X position of the transform'),
    DeclareLaunchArgument('y', default_value='0', description='Y position of the transform'),
    DeclareLaunchArgument('z', default_value='0', description='Z position of the transform'),
    DeclareLaunchArgument('roll', default_value='0', description='Roll angle of the transform'),
    DeclareLaunchArgument('pitch', default_value='0', description='Pitch angle of the transform'),
    DeclareLaunchArgument('yaw', default_value='0', description='Yaw angle of the transform'),
    DeclareLaunchArgument('robot_frame', default_value='base_link', description='Parent frame ID'),
    DeclareLaunchArgument('laser_frame', default_value='laser', description='Child frame ID')
]
    

def generate_launch_description():
    # Lidar node
    sllidar_launch_file = PathJoinSubstitution([get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file),
        launch_arguments={
                'frame_id': LaunchConfiguration('laser_frame'),
                'serial_port': '/dev/ttyUSB1',
                'scan_mode': 'Standard'
            }.items()
    )

    # TF static transform publisher node
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'),
            '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', LaunchConfiguration('robot_frame'),
            '--child-frame-id', LaunchConfiguration('laser_frame')
        ],
        output='screen'
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'),
            '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', "base_link_debug",
            '--child-frame-id', LaunchConfiguration('laser_frame')
        ],
        output='screen'
    )

    # Filter node
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='filter',
        output='screen'
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sllidar_launch)
    ld.add_action(tf_node)
    ld.add_action(lidar_filter_node)
    return ld
