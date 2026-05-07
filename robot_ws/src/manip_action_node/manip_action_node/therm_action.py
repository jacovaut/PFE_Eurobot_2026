#!/usr/bin/env python3
"""Action server for the thermal/mechanical approach motion."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from custom_msgs.action import Therm


class ThermActionServer(Node):
    """Rotates, strafes, then drives forward using timed velocity commands."""

    LEFT = -1
    RIGHT = 1

    def __init__(self):
        super().__init__("therm_action_server")

        self.declare_parameter("action_name", "therm")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_match_input")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("yaw_degrees", 90.0)
        self.declare_parameter("strafe_distance_m", 0.10)
        self.declare_parameter("forward_distance_m", 0.50)
        self.declare_parameter("angular_speed_rad_s", 0.6)
        self.declare_parameter("linear_speed_m_s", 0.15)

        action_name = self.get_parameter("action_name").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.yaw_radians = math.radians(
            float(self.get_parameter("yaw_degrees").value)
        )
        self.strafe_distance = float(
            self.get_parameter("strafe_distance_m").value
        )
        self.forward_distance = float(
            self.get_parameter("forward_distance_m").value
        )
        self.angular_speed = abs(
            float(self.get_parameter("angular_speed_rad_s").value)
        )
        self.linear_speed = abs(
            float(self.get_parameter("linear_speed_m_s").value)
        )

        cb_group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.action_server = ActionServer(
            self,
            Therm,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"[THERM ACTION READY] action={action_name}, topic={cmd_vel_topic}"
        )

    def goal_callback(self, goal_request):
        if goal_request.direction not in (self.LEFT, self.RIGHT):
            self.get_logger().warn(
                "[THERM] Rejecting goal; direction must be -1 LEFT or 1 RIGHT"
            )
            return GoalResponse.REJECT

        side = "right" if goal_request.direction == self.RIGHT else "left"
        self.get_logger().info(f"[THERM] Goal received: side={side}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("[THERM] Cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        direction = goal_handle.request.direction
        started = time.time()

        steps = [
            (
                "rotating",
                self.yaw_radians / self.angular_speed,
                self._twist(angular_z=-direction * self.angular_speed),
            ),
            (
                "strafing",
                self.strafe_distance / self.linear_speed,
                self._twist(linear_y=-direction * self.linear_speed),
            ),
            (
                "forward",
                self.forward_distance / self.linear_speed,
                self._twist(linear_x=self.linear_speed),
            ),
        ]

        total_duration = sum(step[1] for step in steps)
        if self._timed_out(goal_handle.request.timeout_sec, started):
            goal_handle.abort()
            return self._result(False, "Timed out before motion started")

        elapsed_before_step = 0.0
        for state, duration, cmd in steps:
            ok, message = self._run_step(
                goal_handle,
                state,
                cmd,
                duration,
                total_duration,
                elapsed_before_step,
                started,
            )
            if not ok:
                self._stop()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(False, message)
            elapsed_before_step += duration

        self._stop()
        goal_handle.succeed()
        self.get_logger().info("[THERM] Motion complete")
        return self._result(True, "Therm motion complete")

    def _run_step(
        self,
        goal_handle,
        state,
        cmd,
        duration,
        total_duration,
        elapsed_before_step,
        action_started,
    ):
        step_started = time.time()
        period = 1.0 / self.control_rate

        while rclpy.ok() and time.time() - step_started < duration:
            if goal_handle.is_cancel_requested:
                return False, "Canceled"

            timeout_sec = goal_handle.request.timeout_sec
            if self._timed_out(timeout_sec, action_started):
                return False, "Timed out"

            elapsed_in_step = time.time() - step_started
            progress = (elapsed_before_step + elapsed_in_step) / total_duration

            feedback = Therm.Feedback()
            feedback.state = state
            feedback.progress = max(0.0, min(progress, 1.0))
            goal_handle.publish_feedback(feedback)

            self.cmd_pub.publish(cmd)
            time.sleep(period)

        return True, ""

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _timed_out(timeout_sec, started):
        return timeout_sec > 0.0 and time.time() - started > timeout_sec

    @staticmethod
    def _twist(linear_x=0.0, linear_y=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        return cmd

    @staticmethod
    def _result(success, message):
        result = Therm.Result()
        result.success = success
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ThermActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
