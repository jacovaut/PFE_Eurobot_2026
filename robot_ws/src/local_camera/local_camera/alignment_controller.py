#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import rclpy.duration
import rclpy.time


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class AlignmentController(Node):

    def __init__(self):
        super().__init__('alignment_controller')

        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('k_x',   2.5)
        self.declare_parameter('k_y',   2.5)
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_ax', 1.5)
        self.declare_parameter('max_ay', 1.5)
        self.declare_parameter('tol_xy',  0.01)

        self.dt     = 1.0 / self.get_parameter('control_rate').value
        self.kx     = self.get_parameter('k_x').value
        self.ky     = self.get_parameter('k_y').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_ax  = self.get_parameter('max_ax').value
        self.max_ay  = self.get_parameter('max_ay').value
        self.tol_xy  = self.get_parameter('tol_xy').value

        self.last_vx = 0.0
        self.last_vy = 0.0

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_smoothed", 10)

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("[ALIGNMENT CONTROLLER READY]")

    def loop(self):
        timeout = rclpy.duration.Duration(seconds=0.005)
        try:
            tf_cup = self.tf_buffer.lookup_transform(
                "base_link", "cup_0", rclpy.time.Time(), timeout=timeout
            )
            tf_block = self.tf_buffer.lookup_transform(
                "base_link", "block_target_0", rclpy.time.Time(), timeout=timeout
            )
        except Exception:
            self._publish_stop()
            return

        dx = tf_block.transform.translation.x - tf_cup.transform.translation.x
        dy = tf_block.transform.translation.y - tf_cup.transform.translation.y

        if (dx * dx + dy * dy) ** 0.5 < self.tol_xy:
            self._publish_stop()
            return

        # Proportional control
        vx = clamp(self.kx * dx, -self.max_vx, self.max_vx)
        vy = clamp(self.ky * dy, -self.max_vy, self.max_vy)

        # Acceleration limiting (smoothing)
        max_dvx = self.max_ax * self.dt
        max_dvy = self.max_ay * self.dt
        vx = self.last_vx + clamp(vx - self.last_vx, -max_dvx, max_dvx)
        vy = self.last_vy + clamp(vy - self.last_vy, -max_dvy, max_dvy)

        self.last_vx = vx
        self.last_vy = vy

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.last_vx = 0.0
        self.last_vy = 0.0


def main():
    rclpy.init()
    node = AlignmentController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
