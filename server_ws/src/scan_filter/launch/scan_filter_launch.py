"""
scan_filter_launch.py
---------------------
Launches the scan_boundary_filter_node that clips the lidar PointCloud2 to the
Eurobot game table boundaries before it reaches Nav2's costmap.

Run standalone:
  ros2 launch scan_filter scan_filter_launch.py

Or include it in a parent launch file:
  IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('scan_filter'), 'launch', 'scan_filter_launch.py')
      )
  )
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic',  default_value='/scan_cloud'),
        DeclareLaunchArgument('output_topic', default_value='/scan_cloud_filtered'),
        DeclareLaunchArgument('map_frame',    default_value='map'),
        DeclareLaunchArgument('map_x_min',    default_value='0.0'),
        DeclareLaunchArgument('map_x_max',    default_value='3.0'),
        DeclareLaunchArgument('map_y_min',    default_value='0.0'),
        DeclareLaunchArgument('map_y_max',    default_value='2.0'),
        DeclareLaunchArgument('margin',       default_value='0.05'),

        Node(
            package='scan_filter',
            executable='scan_boundary_filter',
            name='scan_boundary_filter_node',
            output='log',
            parameters=[{
                'input_topic':  LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'map_frame':    LaunchConfiguration('map_frame'),
                'map_x_min':    LaunchConfiguration('map_x_min'),
                'map_x_max':    LaunchConfiguration('map_x_max'),
                'map_y_min':    LaunchConfiguration('map_y_min'),
                'map_y_max':    LaunchConfiguration('map_y_max'),
                'margin':       LaunchConfiguration('margin'),
            }],
        ),
    ])
