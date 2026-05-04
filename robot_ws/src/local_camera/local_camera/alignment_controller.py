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

    # =========================
    # GOAL CALLBACK
    # =========================
    def goal_callback(self, msg: Pose2D):
        if not self.active:
            self.get_logger().info(
                f"[NEW GOAL] dx={msg.x:.3f}, dy={msg.y:.3f}, yaw={math.degrees(msg.theta):.1f} deg"
            )
            self.active = True
            self.stable_start = None

        # Always update goal with latest pose estimate (dynamic docking behavior)
        self.goal = msg

    # =========================
    # LOOP
    # =========================
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

        dx = self.goal.x
        dy = self.goal.y
        dtheta = self.goal.theta

        # ===== CHECK TOLERANCE =====
        in_pos = abs(dx) < self.tol_xy and abs(dy) < self.tol_xy
        in_ang = abs(dtheta) < self.tol_theta

        if in_pos and in_ang:
            if self.stable_start is None:
                self.stable_start = time.time()

            if time.time() - self.stable_start > self.stable_time_required:
                self.get_logger().info("[ALIGNMENT COMPLETE]")

                self.publish_stop()

                msg = String()
                msg.data = "aligned"
                self.status_pub.publish(msg)

                self.active = False
                self.goal = None
                return
        else:
            self.stable_start = None

        # ===== CONTROL LAW =====
        vx = self.kx * dx
        vy = self.ky * dy
        w = self.kt * dtheta

        # ===== LIMIT VELOCITY =====
        vx = clamp(vx, -self.max_vx, self.max_vx)
        vy = clamp(vy, -self.max_vy, self.max_vy)
        w = clamp(w, -self.max_w, self.max_w)

        # ===== LIMIT ACCELERATION =====
        vx = self.limit_accel(vx, self.last_cmd.linear.x, self.max_ax)
        vy = self.limit_accel(vy, self.last_cmd.linear.y, self.max_ay)
        w = self.limit_accel(w, self.last_cmd.angular.z, self.max_aw)

        # ===== PUBLISH =====
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
