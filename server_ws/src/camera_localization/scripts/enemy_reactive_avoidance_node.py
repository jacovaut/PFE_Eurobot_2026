#!/usr/bin/env python3
"""
Enemy reactive avoidance node.

Sits at the end of the cmd_vel pipeline, after the collision monitor.
When the enemy robot is detected within reactive_distance_m of the robot,
this node overrides cmd_vel with a holonomic velocity that moves the robot
directly away from the enemy (taking advantage of mecanum wheels).
When the enemy is clear, cmd_vel is passed through unchanged.

Pipeline (requires collision_monitor.cmd_vel_out_topic: "cmd_vel_nav"):
  nav2 → cmd_vel_smoothed → collision_monitor → cmd_vel_nav
                                                      |
                                      enemy_reactive_avoidance
                                                      |
                                                 cmd_vel → robot
"""
import json
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import tf2_ros


class EnemyReactiveAvoidance(Node):
    def __init__(self):
        super().__init__('enemy_reactive_avoidance')

        self.reactive_dist = self.declare_parameter('reactive_distance_m', 0.60).value
        self.speed = self.declare_parameter('reactive_speed_m_s', 0.25).value
        self.cooldown_s = self.declare_parameter('cooldown_s', 0.5).value
        self.enemy_stale_s = self.declare_parameter('enemy_stale_s', 1.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self._nav_cb, 10)
        self.create_subscription(String, '/detected_enemy', self._enemy_cb, 10)

        self._nav_cmd = Twist()
        self._enemy_x: float | None = None
        self._enemy_y: float | None = None
        self._last_enemy_t = None
        self._backing = False
        self._cooldown_until = None

        self.create_timer(0.05, self._tick)  # 20 Hz reactive loop

    def _enemy_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return
        if data:
            e = data[0]
            self._enemy_x = float(e.get('x', 0.0))
            self._enemy_y = float(e.get('y', 0.0))
            self._last_enemy_t = self.get_clock().now()

    def _nav_cb(self, msg: Twist) -> None:
        self._nav_cmd = msg
        if not self._backing:
            self._pub.publish(msg)

    def _robot_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return tx, ty, yaw

    def _tick(self) -> None:
        now = self.get_clock().now()

        enemy_fresh = (
            self._enemy_x is not None
            and self._last_enemy_t is not None
            and (now - self._last_enemy_t).nanoseconds * 1e-9 < self.enemy_stale_s
        )

        should_back = False
        back_cmd = Twist()

        if enemy_fresh:
            pose = self._robot_in_map()
            if pose is not None:
                rx, ry, ryaw = pose
                dx = self._enemy_x - rx
                dy = self._enemy_y - ry
                dist = math.hypot(dx, dy)

                if dist < self.reactive_dist:
                    should_back = True
                    # Unit vector pointing away from enemy, in map frame
                    if dist > 0.01:
                        ax_map = -dx / dist
                        ay_map = -dy / dist
                    else:
                        ax_map, ay_map = -1.0, 0.0
                    # Rotate from map frame into base_link frame
                    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
                    back_cmd.linear.x = (cos_y * ax_map + sin_y * ay_map) * self.speed
                    back_cmd.linear.y = (-sin_y * ax_map + cos_y * ay_map) * self.speed

        if should_back:
            if not self._backing:
                self.get_logger().info(
                    f'Enemy within {self.reactive_dist:.2f} m — backing away'
                )
            self._backing = True
            self._cooldown_until = None
            self._pub.publish(back_cmd)
        elif self._backing:
            if self._cooldown_until is None:
                self._cooldown_until = now + Duration(seconds=self.cooldown_s)
            if now < self._cooldown_until:
                self._pub.publish(Twist())
            else:
                self.get_logger().info('Enemy clear — resuming navigation')
                self._backing = False
                self._cooldown_until = None
                self._pub.publish(self._nav_cmd)
        # passthrough while not backing is handled inline in _nav_cb


if __name__ == '__main__':
    rclpy.init()
    node = EnemyReactiveAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()
