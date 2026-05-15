
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import os
import yaml


def _normalize_team_color(color):
    color = str(color).strip().lower()
    if color in ('blue', 'bleu'):
        return 'blue'
    if color in ('yellow', 'jaune'):
        return 'yellow'
    return 'blue'


def _read_team_color_default():
    try:
        camera_share = get_package_share_directory('camera_localization')
        camera_map = os.path.join(camera_share, 'config', 'camera_global_map.yaml')
        with open(camera_map, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        params = data.get('global_localization_node', {}).get('ros__parameters', {})
        return _normalize_team_color(params.get('team_color', 'blue'))
    except (PackageNotFoundError, OSError, yaml.YAMLError):
        return 'blue'


def _scan_filter_launch(enable_condition):
    try:
        scan_filter_share = get_package_share_directory('scan_filter')
    except PackageNotFoundError:
        return []

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_filter_share, 'launch', 'scan_filter_launch.py')
            ),
            launch_arguments={
                'input_topic': '/scan_cloud',
                'output_topic': '/scan_cloud_filtered',
            }.items(),
            condition=enable_condition,
        )
    ]


def generate_launch_description():
    pkg_path = get_package_share_directory('match')
    local_camera_share = get_package_share_directory('local_camera')
    default_team_color = _read_team_color_default()
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

    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value=default_team_color,
        description='Team color for local pickup scoring (blue|yellow, bleu|jaune).',
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file),
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )
    lidar_condition = IfCondition(LaunchConfiguration('enable_lidar'))

    return LaunchDescription([
        enable_lidar_arg,
        team_color_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='log'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='log',
        ),
        Node(
            package='local_camera',
            executable='dock_action_server',
            name='dock_action_server',
            output='log',
        ),
        Node(
            package='local_camera',
            executable='ros_node',
            name='merged_local_pickup_node',
            output='log',
            parameters=[{
                'udp_port': 5005,
                'team_color': LaunchConfiguration('team_color'),
            }],
        ),
        lidar_launch,
        *_scan_filter_launch(lidar_condition),
    ])
