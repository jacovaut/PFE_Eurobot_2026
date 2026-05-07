
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('pfe')
    local_camera_share = get_package_share_directory('local_camera')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    lidar_launch_file = os.path.join(local_camera_share, 'launch', 'ydlidar_pointcloud.launch.py')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_file]),
        value_type=str
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='Start the YDLidar driver and LaserScan to PointCloud2 bridge.',
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file),
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    return LaunchDescription([
        enable_lidar_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='local_camera',
            executable='dock_action_server',
            name='dock_action_server',
            output='screen',
        ),
        Node(
            package='local_camera',
            executable='ros_node',
            name='merged_local_pickup_node',
            output='screen',
            parameters=[{
                'udp_port': 5005,
            }],
        ),
        lidar_launch,
    ])
