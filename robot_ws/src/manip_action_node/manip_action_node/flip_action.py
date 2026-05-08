#!/usr/bin/env python3
"""Action server that sends flip slot commands to the manipulator node."""

import time
from threading import Lock

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8

from custom_msgs.action import Flip
from custom_msgs.msg import Blocks


STATUS_FLIP_STARTED = 4
STATUS_FLIP_DONE = 5
STATUS_FLIP_FAILED = 6


class FlipActionServer(Node):
    """Publishes flip slot commands on the existing manip Blocks topic."""

    VALID_SLOT_STATES = {0, 1}

    def __init__(self):
        super().__init__("flip_action_server")

        self.declare_parameter("action_name", "flip")
        self.declare_parameter("blocks_topic", "/manip_node/flip")
        self.declare_parameter("status_topic", "/manip_node/status")
        self.declare_parameter("publish_repeats", 3)
        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("command_timeout_sec", 30.0)

        action_name = self.get_parameter("action_name").value
        blocks_topic = self.get_parameter("blocks_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.publish_repeats = int(self.get_parameter("publish_repeats").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self._status_lock = Lock()
        self._status_counter = 0
        self._last_status = None

        cb_group = ReentrantCallbackGroup()
        self.blocks_pub = self.create_publisher(Blocks, blocks_topic, 10)
        self.create_subscription(
            UInt8,
            status_topic,
            self._status_callback,
            10,
            callback_group=cb_group,
        )
        self.action_server = ActionServer(
            self,
            Flip,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"[FLIP ACTION READY] action={action_name}, topic={blocks_topic}, status={status_topic}"
        )

    def _status_callback(self, msg: UInt8):
        with self._status_lock:
            self._status_counter += 1
            self._last_status = msg.data

    def goal_callback(self, goal_request):
        colors = list(goal_request.colors)
        if any(color not in {0, 1, 2} for color in colors):
            self.get_logger().warn(
                f"[FLIP] Rejecting invalid colors={colors}; expected 0, 1, or 2"
            )
            return GoalResponse.REJECT

        present_count = sum(1 for color in colors if color != 0)
        if goal_request.count != present_count:
            self.get_logger().warn(
                f"[FLIP] Rejecting count={goal_request.count}; "
                f"colors indicate {present_count} requested flips"
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f"[FLIP] Goal received: colors={colors}, count={goal_request.count}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("[FLIP] Cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = Flip.Feedback()
        feedback.count = goal.count
        feedback.state = "publishing"

        started = time.time()
        deadline = started + (goal.timeout_sec if goal.timeout_sec > 0.0 else self.command_timeout)
        msg = Blocks()
        msg.colors = list(goal.colors)
        msg.count = goal.count

        with self._status_lock:
            start_counter = self._status_counter

        for _ in range(max(self.publish_repeats, 1)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, "Canceled", goal.count)

            self.blocks_pub.publish(msg)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.publish_period)

        feedback.state = "waiting_for_manip"
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, "Canceled", goal.count)

            with self._status_lock:
                seen_new_status = self._status_counter > start_counter
                last_status = self._last_status

            if seen_new_status and last_status == STATUS_FLIP_DONE:
                goal_handle.succeed()
                self.get_logger().info(
                    f"[FLIP] Manipulator completed colors={list(goal.colors)}, count={goal.count}"
                )
                return self._result(True, "Flip completed", goal.count)

            if seen_new_status and last_status == STATUS_FLIP_FAILED:
                goal_handle.abort()
                return self._result(False, "Manipulator reported flip failure", goal.count)

            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        goal_handle.abort()
        return self._result(False, "Timed out waiting for manipulator completion", goal.count)

    @staticmethod
    def _result(success, message, count):
        result = Flip.Result()
        result.success = success
        result.message = message
        result.count = count
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FlipActionServer()
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