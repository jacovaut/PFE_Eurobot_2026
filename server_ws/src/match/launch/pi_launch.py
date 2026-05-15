import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ekf_share = get_package_share_directory('ekf_bringup')
    nav2_share = get_package_share_directory('nav2')

    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw_deg = LaunchConfiguration('start_yaw_deg')
    stamp_diagnostics = LaunchConfiguration('stamp_diagnostics')
    max_stamp_offset_sec = LaunchConfiguration('max_stamp_offset_sec')
    max_tick_delta = LaunchConfiguration('max_tick_delta')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('stamp_diagnostics', default_value='false'),
        DeclareLaunchArgument('max_stamp_offset_sec', default_value='1.0'),
        DeclareLaunchArgument('max_tick_delta', default_value='50000'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_share, 'config', 'nav2_odom.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav2_share, 'config', 'map_config.yaml'),
        ),

        # ticks_listener + ekf1 + ekf2 + initial_pose_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ekf_share, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'start_x': start_x,
                'start_y': start_y,
                'start_yaw_deg': start_yaw_deg,
                'stamp_diagnostics': stamp_diagnostics,
                'max_stamp_offset_sec': max_stamp_offset_sec,
                'max_tick_delta': max_tick_delta,
            }.items(),
        ),

        # Nav2 odom stack (map_server + planners + BT navigator + lifecycle managers)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'odom_nav2.launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'map': map_yaml,
            }.items(),
        ),
    ])
