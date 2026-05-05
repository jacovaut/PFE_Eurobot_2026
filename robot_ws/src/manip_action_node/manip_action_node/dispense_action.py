#!/usr/bin/env python3
"""Action server that asks the manipulator to dispense blocks."""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8

from custom_msgs.action import Dispense


class DispenseActionServer(Node):
    """Publishes the requested dispense count to the manipulator node."""

    def __init__(self):
        super().__init__("dispense_action_server")

        self.declare_parameter("action_name", "dispense")
        self.declare_parameter("dispense_topic", "/manip_node/dispense")
        self.declare_parameter("publish_repeats", 3)
        self.declare_parameter("publish_period", 0.05)

        action_name = self.get_parameter("action_name").value
        dispense_topic = self.get_parameter("dispense_topic").value

        self.publish_repeats = int(self.get_parameter("publish_repeats").value)
        self.publish_period = float(self.get_parameter("publish_period").value)

        cb_group = ReentrantCallbackGroup()
        self.dispense_pub = self.create_publisher(UInt8, dispense_topic, 10)
        self.action_server = ActionServer(
            self,
            Dispense,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"[DISPENSE ACTION READY] action={action_name}, topic={dispense_topic}"
        )

    def goal_callback(self, goal_request):
        if goal_request.count == 0:
            self.get_logger().warn("[DISPENSE] Rejecting goal with count=0")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"[DISPENSE] Goal received: count={goal_request.count}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("[DISPENSE] Cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = Dispense.Feedback()
        feedback.count = goal.count
        feedback.state = "publishing"

        started = time.time()
        msg = UInt8()
        msg.data = goal.count

        for _ in range(max(self.publish_repeats, 1)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, "Canceled", goal.count)

            if goal.timeout_sec > 0.0 and time.time() - started > goal.timeout_sec:
                goal_handle.abort()
                return self._result(False, "Timed out", goal.count)

            self.dispense_pub.publish(msg)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.publish_period)

        goal_handle.succeed()
        self.get_logger().info(f"[DISPENSE] Published count={goal.count}")
        return self._result(True, "Dispense command published", goal.count)

    @staticmethod
    def _result(success, message, count):
        result = Dispense.Result()
        result.success = success
        result.message = message
        result.count = count
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DispenseActionServer()
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
