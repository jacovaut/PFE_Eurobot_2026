#!/usr/bin/env python3
"""Action server that sends picked block states to the manipulator node."""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from custom_msgs.action import Pick
from custom_msgs.msg import Blocks


class PickActionServer(Node):
    """Publishes a Blocks message after docking has completed."""

    VALID_BLOCK_STATES = {0, 1, 2}

    def __init__(self):
        super().__init__("pick_action_server")

        self.declare_parameter("action_name", "pick")
        self.declare_parameter("blocks_topic", "/manip_node/blocks")
        self.declare_parameter("publish_repeats", 3)
        self.declare_parameter("publish_period", 0.05)

        action_name = self.get_parameter("action_name").value
        blocks_topic = self.get_parameter("blocks_topic").value

        self.publish_repeats = int(self.get_parameter("publish_repeats").value)
        self.publish_period = float(self.get_parameter("publish_period").value)

        cb_group = ReentrantCallbackGroup()
        self.blocks_pub = self.create_publisher(Blocks, blocks_topic, 10)
        self.action_server = ActionServer(
            self,
            Pick,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"[PICK ACTION READY] action={action_name}, topic={blocks_topic}"
        )

    def goal_callback(self, goal_request):
        colors = list(goal_request.colors)
        if any(color not in self.VALID_BLOCK_STATES for color in colors):
            self.get_logger().warn(
                f"[PICK] Rejecting invalid colors={colors}; expected 0, 1, or 2"
            )
            return GoalResponse.REJECT

        present_count = sum(1 for color in colors if color != 0)
        if goal_request.count != present_count:
            self.get_logger().warn(
                f"[PICK] Rejecting count={goal_request.count}; "
                f"colors indicate {present_count} present blocks"
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f"[PICK] Goal received: colors={colors}, count={goal_request.count}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("[PICK] Cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = Pick.Feedback()
        feedback.count = goal.count
        feedback.state = "publishing"

        started = time.time()
        msg = Blocks()
        msg.colors = list(goal.colors)
        msg.count = goal.count

        for _ in range(max(self.publish_repeats, 1)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, "Canceled", goal.count)

            if goal.timeout_sec > 0.0 and time.time() - started > goal.timeout_sec:
                goal_handle.abort()
                return self._result(False, "Timed out", goal.count)

            self.blocks_pub.publish(msg)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.publish_period)

        goal_handle.succeed()
        self.get_logger().info(
            f"[PICK] Published colors={list(goal.colors)}, count={goal.count}"
        )
        return self._result(True, "Pick command published", goal.count)

    @staticmethod
    def _result(success, message, count):
        result = Pick.Result()
        result.success = success
        result.message = message
        result.count = count
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
