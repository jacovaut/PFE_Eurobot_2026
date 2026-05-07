import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch_file = os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
    default_params_file = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    params_file_arg = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value=default_params_file,
        description='Path to the YDLidar ROS 2 parameter file.',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic consumed by the point cloud converter.',
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/scan_cloud',
        description='PointCloud2 topic published for Nav2 obstacle sources.',
    )
    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='',
        description='Optional frame override for the published point cloud.',
    )
    z_offset_arg = DeclareLaunchArgument(
        'z_offset',
        default_value='0.0',
        description='Z offset applied to every generated point.',
    )

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file),
        launch_arguments={
            'params_file': LaunchConfiguration('ydlidar_params_file'),
        }.items(),
    )

    pointcloud_node = Node(
        package='local_camera',
        executable='scan_pointcloud_node',
        name='scan_pointcloud_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'output_frame': LaunchConfiguration('output_frame'),
            'z_offset': LaunchConfiguration('z_offset'),
        }],
    )

    return LaunchDescription([
        params_file_arg,
        scan_topic_arg,
        pointcloud_topic_arg,
        output_frame_arg,
        z_offset_arg,
        ydlidar_launch,
        pointcloud_node,
    ])