#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String
import math
import time


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


class AlignmentController(Node):

    def __init__(self):
        super().__init__('alignment_controller')

        # ===== PARAMETERS =====
        self.declare_parameter('k_x', 2.5)
        self.declare_parameter('k_y', 2.5)
        self.declare_parameter('k_theta', 3.0)

        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_w', 2.0)

        self.declare_parameter('max_ax', 1.5)
        self.declare_parameter('max_ay', 1.5)
        self.declare_parameter('max_aw', 6.0)

        self.declare_parameter('tol_xy', 0.01)     # 1 cm
        self.declare_parameter('tol_theta', math.radians(2.0))

        self.declare_parameter('stable_time', 0.3)  # must stay in tolerance
        self.declare_parameter('control_rate', 50.0)

        # ===== LOAD PARAMS =====
        self.kx = self.get_parameter('k_x').value
        self.ky = self.get_parameter('k_y').value
        self.kt = self.get_parameter('k_theta').value

        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_w = self.get_parameter('max_w').value

        self.max_ax = self.get_parameter('max_ax').value
        self.max_ay = self.get_parameter('max_ay').value
        self.max_aw = self.get_parameter('max_aw').value

        self.tol_xy = self.get_parameter('tol_xy').value
        self.tol_theta = self.get_parameter('tol_theta').value

        self.stable_time_required = self.get_parameter('stable_time').value
        self.dt = 1.0 / self.get_parameter('control_rate').value

        # ===== STATE =====
        self.goal = None
        self.active = False
        self.stable_start = None

        self.last_cmd = Twist()

        # ===== ROS =====
        self.create_subscription(Pose2D, "/pickup_pose", self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_smoothed", 10)
        self.status_pub = self.create_publisher(String, "/pickup_status", 10)

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("[ALIGNMENT CONTROLLER READY]")

    # =========================
    # GOAL CALLBACK
    # =========================
    def goal_callback(self, msg: Pose2D):
        if self.active:
            # ignore updates while executing
            return

        self.goal = msg
        self.active = True
        self.stable_start = None

        self.get_logger().info(
            f"[NEW GOAL] dx={msg.x:.3f}, dy={msg.y:.3f}, yaw={math.degrees(msg.theta):.1f} deg"
        )

    # =========================
    # LOOP
    # =========================
    def loop(self):

        if not self.active or self.goal is None:
            self.publish_stop()
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
        cmd.angular.z = w

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

    # =========================
    # HELPERS
    # =========================
    def limit_accel(self, target, current, max_a):
        delta = target - current
        max_delta = max_a * self.dt
        delta = clamp(delta, -max_delta, max_delta)
        return current + delta

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd


# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    node = AlignmentController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()