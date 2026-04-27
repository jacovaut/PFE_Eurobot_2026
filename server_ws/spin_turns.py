#!/usr/bin/env python3
"""
Spin the robot N turns using the Nav2 Spin behavior action.

Usage:
    python3 spin_turns.py <turns> [--ccw]

Arguments:
    turns       Number of full turns (can be fractional, e.g. 0.5, 1, 2.5)
    --ccw       Spin counter-clockwise (default is clockwise)

Examples:
    python3 spin_turns.py 1          # one full clockwise turn
    python3 spin_turns.py 2 --ccw    # two full counter-clockwise turns
    python3 spin_turns.py 0.5        # half turn clockwise
"""

import argparse
import math
import sys

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav2_msgs.action import Spin

# Nav2's Spin behavior clamps target_yaw internally, so we split into
# individual 2π chunks to reliably execute multiple full rotations.
MAX_SINGLE_SPIN_RAD = 2 * math.pi  # one full turn per action call


class SpinNode(Node):
    def __init__(self):
        super().__init__("spin_turns")
        self._client = ActionClient(self, Spin, "spin")
        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _spin_once(self, yaw_rad: float) -> bool:
        """Send a single Spin goal and wait for completion. Returns True on success."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Spin action server not available. Is Nav2 running?")
            return False

        goal = Spin.Goal()
        goal.target_yaw = float(yaw_rad)

        self.get_logger().info(
            f"  Spinning {math.degrees(abs(yaw_rad)):.1f}° "
            f"({'CCW' if yaw_rad > 0 else 'CW'})"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Spin goal rejected by Nav2.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        # action_msgs/GoalStatus: 4 = SUCCEEDED
        self._stop_robot()
        if status == 4:
            return True
        else:
            self.get_logger().warn(f"Spin ended with status {status}.")
            return False

    def _stop_robot(self):
        """Publish zero velocity several times to flush the velocity smoother."""
        stop = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop)
            time.sleep(0.05)

    def spin_turns(self, turns: float, clockwise: bool) -> bool:
        sign = -1.0 if clockwise else 1.0
        total_rad = abs(turns) * 2 * math.pi

        direction_str = "CW" if clockwise else "CCW"
        self.get_logger().info(
            f"Starting {turns} turn(s) {direction_str} "
            f"(total {math.degrees(total_rad):.1f}°)"
        )

        remaining = total_rad
        chunk_index = 0

        while remaining > 1e-6:
            chunk = min(remaining, MAX_SINGLE_SPIN_RAD)
            chunk_index += 1
            self.get_logger().info(f"Chunk {chunk_index}: {math.degrees(chunk):.1f}°")

            if not self._spin_once(sign * chunk):
                self.get_logger().error(f"Spin failed on chunk {chunk_index}.")
                return False

            remaining -= chunk

        self.get_logger().info("All turns completed successfully.")
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Spin the robot N turns via Nav2 Spin behavior.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("turns", type=float, help="Number of turns (e.g. 1, 2.5)")
    parser.add_argument(
        "--ccw", action="store_true", help="Spin counter-clockwise (default: clockwise)"
    )
    args = parser.parse_args()

    if args.turns <= 0:
        parser.error("turns must be a positive number.")

    rclpy.init()
    node = SpinNode()
    try:
        success = node.spin_turns(turns=args.turns, clockwise=not args.ccw)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
