#!/usr/bin/env python3
"""
Send a NavigateToPose goal to Nav2.

Usage:
    python3 send_goal.py <x> <y> [angle_degrees]

Examples:
    python3 send_goal.py 1.0 0.5
    python3 send_goal.py 1.0 0.5 90
    python3 send_goal.py -x 1.0 -y 0.5 -a 45
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw_rad: float) -> tuple:
    """Convert a yaw angle (radians) to a quaternion (x, y, z, w)."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return 0.0, 0.0, sy, cy  # x, y, z, w


class GoalSender(Node):
    def __init__(self):
        super().__init__("goal_sender")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, x: float, y: float, yaw_deg: float = 0.0):
        self.get_logger().info(f"Waiting for navigate_to_pose action server...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after 5 s. Is Nav2 running?")
            return False

        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal -> x={x:.3f}  y={y:.3f}  yaw={yaw_deg:.1f}°"
        )

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        # action_msgs/GoalStatus: 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info("Goal reached successfully.")
            return True
        else:
            self.get_logger().warn(f"Navigation ended with status {status}.")
            return False


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a NavigateToPose goal to Nav2.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("-x", type=float, default=None, help="X position (metres)")
    parser.add_argument("-y", type=float, default=None, help="Y position (metres)")
    parser.add_argument(
        "-a", "--angle", type=float, default=0.0,
        help="Heading angle in degrees (default: 0)",
    )

    # Also support positional: send_goal.py x y [angle]
    parser.add_argument("pos_x", nargs="?", type=float, default=None)
    parser.add_argument("pos_y", nargs="?", type=float, default=None)
    parser.add_argument("pos_angle", nargs="?", type=float, default=None)

    args = parser.parse_args()

    # Positional args take precedence when named flags are absent
    x = args.x if args.x is not None else args.pos_x
    y = args.y if args.y is not None else args.pos_y
    angle = args.angle
    if args.pos_angle is not None and args.angle == 0.0:
        angle = args.pos_angle

    if x is None or y is None:
        parser.error("You must provide at least x and y values.")

    return x, y, angle


def main():
    x, y, angle = parse_args()

    rclpy.init()
    node = GoalSender()
    try:
        success = node.send_goal(x, y, angle)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
