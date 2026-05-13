#!/usr/bin/env python3
"""Final cmd_vel gate controlled by /match/running and close lidar returns."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CmdVelMatchGate(Node):
    """Pass velocity commands only while the match is running and lidar is clear."""

    def __init__(self):
        super().__init__('cmd_vel_match_gate')

        self.declare_parameter('input_topic', '/cmd_vel_match_input')
        self.declare_parameter('output_topic', '/cmd_vel_smoothed')
        self.declare_parameter('match_running_topic', '/match/running')
        self.declare_parameter('zero_publish_hz', 20.0)
        self.declare_parameter('lidar_stop_enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lidar_stop_distance_m', 0.40)
        self.declare_parameter('lidar_clear_distance_m', 0.45)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        match_topic = self.get_parameter('match_running_topic').value
        zero_publish_hz = float(self.get_parameter('zero_publish_hz').value)
        self._lidar_enabled = bool(self.get_parameter('lidar_stop_enabled').value)
        scan_topic = self.get_parameter('scan_topic').value
        self._stop_distance = float(self.get_parameter('lidar_stop_distance_m').value)
        self._clear_distance = max(
            float(self.get_parameter('lidar_clear_distance_m').value),
            self._stop_distance,
        )

        match_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._running = False
        self._lidar_blocked = False
        self._zero = Twist()
        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_cmd_vel, 10)
        self.create_subscription(Bool, match_topic, self._on_match_running, match_qos)
        if self._lidar_enabled:
            self.create_subscription(
                LaserScan,
                scan_topic,
                self._on_scan,
                qos_profile_sensor_data,
            )

        zero_period = 1.0 / max(zero_publish_hz, 1.0)
        self.create_timer(zero_period, self._publish_zero_if_blocked)
        self._pub.publish(self._zero)

        lidar_status = (
            f'; lidar stop on {scan_topic} at {self._stop_distance:.2f} m'
            if self._lidar_enabled
            else '; lidar stop disabled'
        )
        self.get_logger().info(
            f'Gating {input_topic} -> {output_topic}; blocked until {match_topic} is true'
            f'{lidar_status}.'
        )

    def _on_cmd_vel(self, msg: Twist):
        if self._can_pass_cmd_vel():
            self._pub.publish(msg)

    def _on_match_running(self, msg: Bool):
        was_running = self._running
        self._running = bool(msg.data)
        if was_running and not self._running:
            self._pub.publish(self._zero)
            self.get_logger().info('Match stopped; cmd_vel output is hard-blocked.')
        elif self._running and not was_running:
            self.get_logger().info('Match running; cmd_vel output is enabled.')

    def _on_scan(self, msg: LaserScan):
        closest = self._closest_valid_range(msg)
        was_blocked = self._lidar_blocked

        if closest is None:
            self._lidar_blocked = False
        elif closest <= self._stop_distance:
            self._lidar_blocked = True
        elif self._lidar_blocked and closest >= self._clear_distance:
            self._lidar_blocked = False

        if self._lidar_blocked and not was_blocked:
            self._pub.publish(self._zero)
            self.get_logger().warn(
                f'Lidar obstacle at {closest:.2f} m; cmd_vel output is stopped.'
            )
        elif was_blocked and not self._lidar_blocked:
            if closest is None:
                self.get_logger().info('Lidar clear; no valid scan returns in stop zone.')
            else:
                self.get_logger().info(
                    f'Lidar clear at {closest:.2f} m; cmd_vel output may resume.'
                )

    def _closest_valid_range(self, msg: LaserScan):
        range_min = max(float(msg.range_min), 0.0)
        range_max = float(msg.range_max)
        closest = None

        for distance in msg.ranges:
            if not math.isfinite(distance):
                continue
            if distance < range_min:
                continue
            if range_max > 0.0 and distance > range_max:
                continue
            if closest is None or distance < closest:
                closest = distance

        return closest

    def _can_pass_cmd_vel(self):
        return self._running and not self._lidar_blocked

    def _publish_zero_if_blocked(self):
        if not self._can_pass_cmd_vel():
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
