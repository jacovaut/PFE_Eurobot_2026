from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_camera_device = LaunchConfiguration('local_camera_device')
    local_camera_mode = LaunchConfiguration('local_camera_mode')
    local_stream_url = LaunchConfiguration('local_stream_url')
    local_show_debug_window = LaunchConfiguration('local_show_debug_window')

    return LaunchDescription([
        DeclareLaunchArgument(
            'local_camera_device',
            default_value='4',
            description='Camera index for local_perception_node when using device mode'
        ),
        DeclareLaunchArgument(
            'local_camera_mode',
            default_value='device',
            description='Input mode for local_perception_node: device or stream'
        ),
        DeclareLaunchArgument(
            'local_stream_url',
            default_value='tcp://127.0.0.1:8888',
            description='Stream URL for local_perception_node when using stream mode'
        ),
        DeclareLaunchArgument(
            'local_show_debug_window',
            default_value='false',
            description='Show debug window for local_perception_node'
        ),

        Node(
            package='local_camera',
            executable='local_perception_node',
            name='local_camera_perception_node',
            output='screen',
            parameters=[{
                'camera_device': local_camera_device,
                'camera_mode': local_camera_mode,
                'stream_url': local_stream_url,
                'show_debug_window': local_show_debug_window,
            }],
        ),
    ])
