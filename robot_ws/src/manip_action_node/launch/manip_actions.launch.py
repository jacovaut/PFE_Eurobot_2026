from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="manip_action_node",
            executable="pick_action",
            name="pick_action_server",
            output="screen",
        ),
        Node(
            package="manip_action_node",
            executable="dispense_action",
            name="dispense_action_server",
            output="screen",
        ),
        Node(
            package="manip_action_node",
            executable="therm_action",
            name="therm_action_server",
            output="screen",
        ),
    ])
