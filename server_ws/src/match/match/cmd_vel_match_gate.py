#!/usr/bin/env python3
"""Final cmd_vel gate controlled by /match/running."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelMatchGate(Node):
    """Pass velocity commands only while the match is running."""

    def __init__(self):
        super().__init__('cmd_vel_match_gate')

        self.declare_parameter('input_topic', '/cmd_vel_match_input')
        self.declare_parameter('output_topic', '/cmd_vel_smoothed')
        self.declare_parameter('match_running_topic', '/match/running')
        self.declare_parameter('zero_publish_hz', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        match_topic = self.get_parameter('match_running_topic').value
        zero_publish_hz = float(self.get_parameter('zero_publish_hz').value)

        match_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._running = False
        self._zero = Twist()
        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_cmd_vel, 10)
        self.create_subscription(Bool, match_topic, self._on_match_running, match_qos)

        zero_period = 1.0 / max(zero_publish_hz, 1.0)
        self.create_timer(zero_period, self._publish_zero_if_blocked)
        self._pub.publish(self._zero)

        self.get_logger().info(
            f'Gating {input_topic} -> {output_topic}; blocked until {match_topic} is true.'
        )

    def _on_cmd_vel(self, msg: Twist):
        if self._running:
            self._pub.publish(msg)

    def _on_match_running(self, msg: Bool):
        was_running = self._running
        self._running = bool(msg.data)
        if was_running and not self._running:
            self._pub.publish(self._zero)
            self.get_logger().info('Match stopped; cmd_vel output is hard-blocked.')
        elif self._running and not was_running:
            self.get_logger().info('Match running; cmd_vel output is enabled.')

    def _publish_zero_if_blocked(self):
        if not self._running:
            self._pub.publish(self._zero)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMatchGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
