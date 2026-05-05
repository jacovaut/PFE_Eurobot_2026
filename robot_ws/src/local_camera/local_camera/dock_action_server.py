#!/usr/bin/env python3
"""
Docking Action Server for block pickup alignment.

Exposes a ROS2 action: custom_msgs/action/DockToBlock
- Subscribes to /pickup_pose (published by ros_node.py)
- Drives the robot via /cmd_vel_smoothed using a P controller
- Reports feedback (dx, dy, yaw_deg, error_m, state) at 50 Hz
- Succeeds when aligned within tolerance for a stable duration
- Fails on timeout or preempt

States:
  0 = SEARCHING  : no /pickup_pose received yet
  1 = CANDIDATE  : receiving poses but lock not confirmed
  2 = LOCKED     : solution locked, driving toward target
  3 = CONVERGING : within 2x tolerance, slowing down
  4 = ALIGNED    : within tolerance, waiting for stable period
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import time

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String, Int32

from custom_msgs.action import DockToBlock


# =========================
# HELPERS
# =========================
def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


# =========================
# DOCKING ACTION SERVER
# =========================
class DockActionServer(Node):

    # States
    STATE_SEARCHING  = 0
    STATE_CANDIDATE  = 1
    STATE_LOCKED     = 2
    STATE_CONVERGING = 3
    STATE_ALIGNED    = 4
    STATE_WAITING    = 5
    STATE_ORBITING   = 6

    def __init__(self):
        super().__init__("dock_action_server")

        # ----- Parameters -----
        self.declare_parameter("k_x",          0.5)
        self.declare_parameter("k_y",          0.5)
        self.declare_parameter("k_theta",      3.0)
        self.declare_parameter("max_vx",       0.4)
        self.declare_parameter("max_vy",       0.7)
        self.declare_parameter("max_w",        0.8)
        self.declare_parameter("max_ax",       1.5)
        self.declare_parameter("max_ay",       2.5)
        self.declare_parameter("max_aw",       6.0)
        self.declare_parameter("tol_xy",              0.025)
        self.declare_parameter("tol_theta",           math.radians(5.0))
        self.declare_parameter("stable_time",         0.3)
        self.declare_parameter("control_rate",        50.0)
        self.declare_parameter("blind_commit_dist",   0.040)  # m: if last error < this when going blind, succeed
        self.declare_parameter("blind_hold_time",     1.0)    # s: how long to hold position waiting for pose to return
        self.declare_parameter("orbit_radius",         0.65)   # m: fallback orbit radius (cup_0 TF used if available)
        self.declare_parameter("orbit_speed",         0.12)   # m/s tangential orbit speed
        self.declare_parameter("orbit_k_r",           3.0)    # radial correction gain
        self.kx    = self.get_parameter("k_x").value
        self.ky    = self.get_parameter("k_y").value
        self.kt    = self.get_parameter("k_theta").value
        self.max_vx = self.get_parameter("max_vx").value
        self.max_vy = self.get_parameter("max_vy").value
        self.max_w  = self.get_parameter("max_w").value
        self.max_ax = self.get_parameter("max_ax").value
        self.max_ay = self.get_parameter("max_ay").value
        self.max_aw = self.get_parameter("max_aw").value
        self.tol_xy    = self.get_parameter("tol_xy").value
        self.tol_theta = self.get_parameter("tol_theta").value
        self.stable_time_required  = self.get_parameter("stable_time").value
        self.blind_commit_dist     = self.get_parameter("blind_commit_dist").value
        self.blind_hold_time       = self.get_parameter("blind_hold_time").value
        self.orbit_radius          = self.get_parameter("orbit_radius").value
        self.orbit_speed           = self.get_parameter("orbit_speed").value
        self.orbit_k_r             = self.get_parameter("orbit_k_r").value
        self.dt = 1.0 / self.get_parameter("control_rate").value

        # ----- State -----
        self.current_pose: Pose2D | None = None
        self.n_blocks: int = 1
        self.pose_stamp: float = 0.0
        self.last_cmd = Twist()
        self.pose_max_age = 0.5       # seconds before pose considered stale
        self.last_good_error: float = float("inf")  # error when pose was last valid
        self.last_good_state: int = self.STATE_SEARCHING

        # ----- ROS interfaces -----
        cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_smoothed", 10)
        self.status_pub = self.create_publisher(String, "/pickup_status", 10)

        self.create_subscription(
            Pose2D,
            "/pickup_pose",
            self._pose_callback,
            10,
            callback_group=cb_group
        )

        self.create_subscription(
            Int32,
            "/pickup_n",
            self._n_callback,
            10,
            callback_group=cb_group
        )

        self._action_server = ActionServer(
            self,
            DockToBlock,
            "dock_to_block",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group
        )

        self.get_logger().info("[DOCK ACTION SERVER READY]")

    # =========================
    # POSE SUBSCRIBER
    # =========================
    def _pose_callback(self, msg: Pose2D):
        self.current_pose = msg
        self.pose_stamp = time.time()

    def _n_callback(self, msg: Int32):
        self.n_blocks = msg.data

    # =========================
    # ACTION CALLBACKS
    # =========================
    def _goal_callback(self, goal_request):
        self.get_logger().info("[DOCK] Goal received")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("[DOCK] Cancel requested")
        return CancelResponse.ACCEPT

    # =========================
    # EXECUTE
    # =========================
    def _execute(self, goal_handle):
        timeout_sec = goal_handle.request.timeout_sec
        start_time  = time.time()
        feedback = DockToBlock.Feedback()
        self.last_good_error = float("inf")
        self.last_good_state = self.STATE_SEARCHING

        self.get_logger().info(
            f"[DOCK] Executing (timeout={timeout_sec:.1f}s)"
        )

        # Reset pickup solver lock so it starts fresh
        self._publish_status("unlock")

        phase        = "approach"           # approach | orbit | approach2
        stable_start: float | None = None   # position/angle stable timer
        blind_start:  float | None = None   # blind-hold timer
        r_target      = 0.0                 # radial distance to maintain during orbit
        orbit_dir     = 1.0                 # +1 = CCW, -1 = CW (locked at orbit start)

        while rclpy.ok():

            # ----- Timeout -----
            elapsed = time.time() - start_time
            if timeout_sec > 0 and elapsed > timeout_sec:
                self._stop()
                goal_handle.abort()
                result = DockToBlock.Result()
                result.success = False
                result.message = f"Timeout after {elapsed:.1f}s"
                result.final_error_m = self._current_error()
                self.get_logger().warn("[DOCK] Timed out")
                return result

            # ----- Preempt -----
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result = DockToBlock.Result()
                result.success = False
                result.message = "Canceled"
                result.final_error_m = self._current_error()
                self.get_logger().info("[DOCK] Canceled")
                return result

            # ----- Get latest pose -----
            pose = self.current_pose
            pose_age = time.time() - self.pose_stamp
            pose_stale = (pose is None or pose_age > self.pose_max_age)

            if pose_stale:
                # ---- Blind phase: blocks left camera FOV ----
                if phase == "approach" and self.last_good_error < self.blind_commit_dist:
                    # We were close enough — commit success immediately
                    self._stop()
                    self._publish_status("aligned")
                    goal_handle.succeed()
                    result = DockToBlock.Result()
                    result.success = True
                    result.message = f"Blind commit (last error={self.last_good_error*1000:.1f}mm)"
                    result.final_error_m = self.last_good_error
                    self.get_logger().info(
                        f"[DOCK] Blind commit — blocks under robot, last error={self.last_good_error*1000:.1f}mm"
                    )
                    return result

                # Blocks went out of view too early — hold and wait
                if blind_start is None:
                    blind_start = time.time()

                if time.time() - blind_start > self.blind_hold_time:
                    # Waited long enough with no blocks — abort
                    self._stop()
                    goal_handle.abort()
                    result = DockToBlock.Result()
                    result.success = False
                    result.message = f"Lost blocks (last error={self.last_good_error*1000:.1f}mm)"
                    result.final_error_m = self.last_good_error
                    self.get_logger().warn("[DOCK] Lost blocks during approach")
                    return result

                self._stop()
                feedback.dx      = 0.0
                feedback.dy      = 0.0
                feedback.yaw_deg = 0.0
                feedback.error_m = self.last_good_error
                feedback.state   = self.STATE_SEARCHING
                goal_handle.publish_feedback(feedback)
                time.sleep(self.dt)
                continue
            else:
                blind_start = None  # reset blind hold timer when pose is valid again

            dx     = pose.x
            dy     = pose.y
            dtheta = pose.theta
            error  = math.hypot(dx, dy)
            self.last_good_error = error

            # ==================
            # PHASE: APPROACH
            # ==================
            if phase == "approach":
                in_pos = abs(dx) < self.tol_xy and abs(dy) < self.tol_xy
                if in_pos:
                    state = self.STATE_ALIGNED
                elif error < self.tol_xy * 2.5:
                    state = self.STATE_CONVERGING
                else:
                    state = self.STATE_LOCKED
                self.last_good_state = state

                if in_pos:
                    vx = vy = 0.0
                    if stable_start is None:
                        stable_start = time.time()
                    if time.time() - stable_start >= self.stable_time_required:
                        self._stop()
                        if self.n_blocks == 1:
                            r_target = self.orbit_radius
                            orbit_dir = 1.0 if dtheta >= 0 else -1.0
                            phase = "orbit"
                            self.get_logger().info(
                                f"[DOCK] Position reached, starting orbit "
                                f"({'CCW' if orbit_dir>0 else 'CW'}, dtheta={math.degrees(dtheta):.1f}°)")
                        else:
                            phase = "approach2"
                            self.get_logger().info(
                                f"[DOCK] Position reached (n={self.n_blocks}), skipping orbit → approach2")
                        stable_start = None
                else:
                    stable_start = None
                    vx = clamp(self.kx * dx, -self.max_vx, self.max_vx)
                    vy = clamp(self.ky * dy, -self.max_vy, self.max_vy)
                w = 0.0

            # ==================
            # PHASE: ORBIT
            # ==================
            elif phase == "orbit":
                state = self.STATE_ORBITING
                self.last_good_state = state

                # Angular error to target orientation (wraps to [-π, π])
                ang_err = dtheta
                in_ang       = abs(ang_err) < self.tol_theta
                in_ang_loose = abs(ang_err) < self.tol_theta * 2.0

                if stable_start is not None:
                    # Already in confirmation window — hold still
                    if not in_ang_loose:
                        # Genuinely drifted back out — restart orbit
                        stable_start = None
                        self.get_logger().info(
                            f"[DOCK] Orbit: drifted back (err={math.degrees(ang_err):.1f}°), resuming orbit")
                    elif time.time() - stable_start >= self.stable_time_required:
                        self._stop()
                        phase = "approach2"
                        stable_start = None
                        self.get_logger().info("[DOCK] Orbit aligned, starting second XY alignment")
                    vx = vy = w = 0.0
                elif in_ang:
                    # Just entered tolerance — start confirmation timer
                    stable_start = time.time()
                    self.get_logger().info(
                        f"[DOCK] Orbit: angle reached (err={math.degrees(ang_err):.1f}°), "
                        f"waiting {self.stable_time_required:.1f}s to confirm")
                    vx = vy = w = 0.0
                else:
                    # Orbit in the locked direction
                    r_orb     = max(r_target, 0.05)
                    omega_orb = self.orbit_speed / r_orb
                    vx = 0.0
                    vy = self.orbit_speed * orbit_dir
                    w  = clamp(-omega_orb * orbit_dir, -self.max_w, self.max_w)

            # ==================
            # PHASE: APPROACH2
            # ==================
            elif phase == "approach2":
                in_pos = abs(dx) < self.tol_xy and abs(dy) < self.tol_xy
                in_ang = abs(dtheta) < self.tol_theta
                if in_pos and in_ang:
                    state = self.STATE_ALIGNED
                elif error < self.tol_xy * 2.5:
                    state = self.STATE_CONVERGING
                else:
                    state = self.STATE_LOCKED
                self.last_good_state = state

                if in_pos and in_ang:
                    vx = vy = w = 0.0
                    if stable_start is None:
                        stable_start = time.time()
                    if time.time() - stable_start >= self.stable_time_required:
                        self._stop()
                        self._publish_status("aligned")
                        goal_handle.succeed()
                        result = DockToBlock.Result()
                        result.success = True
                        result.message = "Final alignment complete"
                        result.final_error_m = error
                        self.get_logger().info("[DOCK] Final alignment complete!")
                        return result
                else:
                    stable_start = None
                    vx = clamp(self.kx * dx, -self.max_vx, self.max_vx)
                    vy = clamp(self.ky * dy, -self.max_vy, self.max_vy)
                    w  = clamp(self.kt * dtheta, -self.max_w, self.max_w)

            # Acceleration limiting
            # Note: cmd.linear.y = -vy, so last_cmd.linear.y = -vy_prev.
            # Un-negate when reading back so the limiter tracks pre-negation values.
            vx = self._limit_accel(vx,  self.last_cmd.linear.x,   self.max_ax)
            vy = self._limit_accel(vy, -self.last_cmd.linear.y,   self.max_ay)
            w  = self._limit_accel(w,   self.last_cmd.angular.z,  self.max_aw)

            cmd = Twist()
            cmd.linear.x  = vx
            cmd.linear.y  = -vy
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd

            # ----- Feedback -----
            feedback.dx      = dx
            feedback.dy      = dy
            feedback.yaw_deg = math.degrees(dtheta)
            feedback.error_m = error
            feedback.state   = state
            goal_handle.publish_feedback(feedback)

            time.sleep(self.dt)

        # Should not reach here
        self._stop()
        goal_handle.abort()
        result = DockToBlock.Result()
        result.success = False
        result.message = "Node shutdown"
        result.final_error_m = self._current_error()
        return result

    # =========================
    # HELPERS
    # =========================
    def _limit_accel(self, target, current, max_a):
        delta = clamp(target - current, -max_a * self.dt, max_a * self.dt)
        return current + delta

    def _stop(self):
        self.cmd_pub.publish(Twist())
        self.last_cmd = Twist()

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _current_error(self) -> float:
        if self.current_pose is None:
            return 0.0
        return math.hypot(self.current_pose.x, self.current_pose.y)


# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    node = DockActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
