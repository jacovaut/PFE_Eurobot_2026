import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _read_cluster_pipeline_default(camera_map_path: str) -> str:
    try:
        with open(camera_map_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        enabled = data.get('global_localization_node', {}).get('ros__parameters', {}).get(
            'use_cluster_pipeline',
            True,
        )
        return 'true' if bool(enabled) else 'false'
    except Exception:
        return 'true'


def _read_team_color_default(camera_map_path: str) -> str:
    try:
        with open(camera_map_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        return str(
            data.get('global_localization_node', {})
            .get('ros__parameters', {})
            .get('team_color', 'blue')
        )
    except Exception:
        return 'blue'


def generate_launch_description():
    ekf_share = get_package_share_directory('ekf_bringup')
    camera_share = get_package_share_directory('camera_localization')
    nav2_share = get_package_share_directory('nav2')

    default_camera_map = os.path.join(camera_share, 'config', 'camera_global_map.yaml')

    ekf1_config = os.path.join(ekf_share, 'config', 'ekf1.yaml')
    ekf2_config = os.path.join(ekf_share, 'config', 'ekf2.yaml')

    camera_global_map_config = LaunchConfiguration('camera_global_map_config')
    use_cluster_pipeline = LaunchConfiguration('use_cluster_pipeline')
    launch_map_visualizer = LaunchConfiguration('launch_map_visualizer')
    publish_block_obstacles = LaunchConfiguration('publish_block_obstacles')
    team_color = LaunchConfiguration('team_color')
    cluster_robot_marker_id = LaunchConfiguration('cluster_robot_marker_id')
    cluster_show_debug_window = LaunchConfiguration('cluster_show_debug_window')
    cluster_goal_min_score = LaunchConfiguration('cluster_goal_min_score')
    cluster_goal_offset_m = LaunchConfiguration('cluster_goal_offset_m')
    cluster_goal_update_period_s = LaunchConfiguration('cluster_goal_update_period_s')

    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw_deg = LaunchConfiguration('start_yaw_deg')
    stamp_diagnostics = LaunchConfiguration('stamp_diagnostics')
    max_stamp_offset_sec = LaunchConfiguration('max_stamp_offset_sec')
    max_tick_delta = LaunchConfiguration('max_tick_delta')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    force_match_running = LaunchConfiguration('force_match_running')
    enable_opener = LaunchConfiguration('enable_opener')

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'smoother_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    launch_arguments = [
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=default_camera_map,
            description='World map + camera config YAML for global camera and map visualizer.',
        ),
        DeclareLaunchArgument(
            'use_cluster_pipeline',
            default_value=_read_cluster_pipeline_default(default_camera_map),
            description='Launch cluster analysis and cluster-to-Nav2 goal bridge.',
        ),
        DeclareLaunchArgument(
            'launch_map_visualizer',
            default_value='true',
            description='Launch camera map visualizer and obstacle publishers.',
        ),
        DeclareLaunchArgument(
            'publish_block_obstacles',
            default_value='true',
            description='Publish camera obstacle point clouds/grids for Nav2.',
        ),
        DeclareLaunchArgument(
            'team_color',
            default_value=_read_team_color_default(default_camera_map),
            description='Team color used by localization, strategy, and enemy nid navigation exclusion.',
        ),
        DeclareLaunchArgument(
            'cluster_robot_marker_id',
            default_value='1',
            description='Legacy cluster robot marker id parameter.',
        ),
        DeclareLaunchArgument(
            'cluster_show_debug_window',
            default_value='true',
            description='Show OpenCV cluster debug window.',
        ),
        DeclareLaunchArgument(
            'cluster_goal_min_score',
            default_value='0.0',
            description='Minimum cluster score required before sending a Nav2 goal.',
        ),
        DeclareLaunchArgument(
            'cluster_goal_offset_m',
            default_value='0.18',
            description='Approach offset from selected cluster center.',
        ),
        DeclareLaunchArgument(
            'cluster_goal_update_period_s',
            default_value='0.7',
            description='Minimum time between cluster goal updates.',
        ),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('stamp_diagnostics', default_value='false'),
        DeclareLaunchArgument('max_stamp_offset_sec', default_value='1.0'),
        DeclareLaunchArgument('max_tick_delta', default_value='50000'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level Nav2 namespace.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav2_share, 'config', 'nav2_full.yaml'),
            description='Nav2 full navigation parameter file.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav2_share, 'config', 'map_config.yaml'),
            description='Map YAML for Nav2 map_server.',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'force_match_running',
            default_value='true',
            description='Force /match/running true for testing outside a match.',
        ),
        DeclareLaunchArgument(
            'enable_opener',
            default_value='false',
            description='Run the stock opener sequence when /match/running becomes true.',
        ),
    ]

    localization_nodes = [
        Node(
            package='deadwheel_odometry',
            executable='ticks_listener',
            name='ticks_listener',
            output='screen',
            parameters=[{
                'initial_x': start_x,
                'initial_y': start_y,
                'initial_yaw_deg': start_yaw_deg,
                'stamp_diagnostics': ParameterValue(stamp_diagnostics, value_type=bool),
                'max_stamp_offset_sec': ParameterValue(max_stamp_offset_sec, value_type=float),
                'max_tick_delta': ParameterValue(max_tick_delta, value_type=int),
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf1_config],
            remappings=[
                ('/odometry/filtered', '/odometry/local'),
                ('set_pose', '/ekf_local_node/set_pose'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[ekf2_config],
            remappings=[
                ('/odometry/filtered', '/odometry/global'),
                ('set_pose', '/ekf_global_node/set_pose'),
            ],
        ),
        Node(
            package='ekf_bringup',
            executable='initial_pose_bridge',
            name='initial_pose_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/initialpose',
                'set_pose_topic': '/ekf_global_node/set_pose',
                'set_pose_service': '/ekf_global_node/set_pose',
                'map_frame': 'map',
            }],
        ),
    ]

    camera_and_strategy_nodes = [
        Node(
            package='camera_localization',
            executable='global_localization_node',
            name='global_localization_node',
            output='screen',
            parameters=[camera_global_map_config, {'team_color': team_color}],
        ),
        Node(
            package='camera_localization',
            executable='camera_map_visualizer_node.py',
            name='camera_map_visualizer_node',
            output='screen',
            emulate_tty=True,
            prefix='python3 -u -X faulthandler',
            condition=IfCondition(launch_map_visualizer),
            parameters=[camera_global_map_config, {
                'publish_block_obstacles': publish_block_obstacles,
                'map_frame': 'map',
                'robot_pose_topic': '/camera/global_pose',
                'detected_blocks_topic': '/detected_blocks',
                'marker_topic': '/camera_map/markers',
                'block_pointcloud_topic': '/camera/block_obstacles',
                'team_color': team_color,
            }],
        ),
        Node(
            package='strategy',
            executable='cluster_analyze_node',
            name='cluster_analyze_node',
            output='screen',
            condition=IfCondition(use_cluster_pipeline),
            parameters=[{
                'team_color': team_color,
                'robot_marker_id': cluster_robot_marker_id,
                'show_debug_window': cluster_show_debug_window,
            }],
        ),
        Node(
            package='camera_localization',
            executable='cluster_goal_bridge_node.py',
            name='cluster_goal_bridge_node',
            output='screen',
            condition=IfCondition(use_cluster_pipeline),
            parameters=[{
                'cluster_topic': '/cluster_info',
                'action_name': 'navigate_to_pose',
                'goal_frame': 'map',
                'enabled': True,
                'min_score': cluster_goal_min_score,
                'approach_offset_m': cluster_goal_offset_m,
                'min_goal_update_period_s': cluster_goal_update_period_s,
            }],
        ),
    ]

    match_and_reactive_nodes = [
        Node(
            package='match',
            executable='match_node',
            name='match_node',
            output='screen',
            parameters=[{
                'autostart': False,
                'duration_s': 98.0,
                'force_running': ParameterValue(force_match_running, value_type=bool),
                'team_color': team_color,
                'enable_opener': ParameterValue(enable_opener, value_type=bool),
            }],
        ),
        Node(
            package='match',
            executable='cmd_vel_match_gate',
            name='cmd_vel_match_gate',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_match_input',
                'output_topic': '/cmd_vel_smoothed',
                'match_running_topic': '/match/running',
                'zero_publish_hz': 20.0,
            }],
        ),
        Node(
            package='camera_localization',
            executable='enemy_reactive_avoidance_node.py',
            name='enemy_reactive_avoidance',
            output='screen',
            parameters=[{
                'reactive_distance_m': 0.50,
                'reactive_speed_m_s': 0.25,
                'cooldown_s': 0.5,
                'enemy_stale_s': 2.0,
                'output_topic': '/cmd_vel_match_input',
            }],
        ),
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_nav2_params, {'yaml_filename': map_yaml}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': ['map_server']}],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_pre_reactive')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_raw'),
                    ('cmd_vel_smoothed', 'cmd_vel_pre_reactive'),
                ],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        *launch_arguments,
        *localization_nodes,
        *camera_and_strategy_nodes,
        *match_and_reactive_nodes,
        nav2_nodes,
    ])
