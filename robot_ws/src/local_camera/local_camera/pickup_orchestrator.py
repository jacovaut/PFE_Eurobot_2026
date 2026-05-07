#!/usr/bin/env python3
"""
pickup_orchestrator.py
----------------------
Chains the DockToBlock and Pick actions:
  1. Waits for DockToBlock to succeed
  2. Reads the last /best_pickup to extract block colors
  3. Sends a Pick goal to the pick_action_server

Run with:
  ros2 run local_camera pickup_orchestrator
"""

import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from custom_msgs.action import DockToBlock, Pick
from .team_color import normalize_team_color, read_default_team_color


class PickupOrchestrator(Node):
    def __init__(self):
        super().__init__("pickup_orchestrator")

        self.declare_parameter("team_color", read_default_team_color())
        self.declare_parameter("dock_timeout_sec", 30.0)
        self.declare_parameter("pick_timeout_sec", 0.0)

        self.team_color = normalize_team_color(
            self.get_parameter("team_color").value
        )
        self.dock_timeout = float(self.get_parameter("dock_timeout_sec").value)
        self.pick_timeout = float(self.get_parameter("pick_timeout_sec").value)

        # Last known assignment from the solver
        self._last_best_pickup: dict | None = None

        self.create_subscription(
            String,
            "/best_pickup",
            self._best_pickup_cb,
            10,
        )

        self._dock_client = ActionClient(self, DockToBlock, "dock_to_block")
        self._pick_client = ActionClient(self, Pick, "pick")
        self.get_logger().info(f"[ORCHESTRATOR READY] team_color={self.team_color}")

    def _color_to_pick_mode(self, color: str) -> int:
        # Pick.Goal colors: 0 = absent, 1 = pick as-is, 2 = pick and flip.
        if str(color).strip().lower() == "unknown":
            return 1
        color = normalize_team_color(color)
        if color == self.team_color:
            return 1
        return 2

    def _best_pickup_cb(self, msg: String) -> None:
        try:
            self._last_best_pickup = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Could not parse /best_pickup JSON")

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Block until the full dock → pick sequence completes."""
        self.get_logger().info("[ORCHESTRATOR] Waiting for action servers…")
        self._dock_client.wait_for_server()
        self._pick_client.wait_for_server()
        self.get_logger().info("[ORCHESTRATOR] Action servers ready.")

        if not self._do_dock():
            self.get_logger().error("[ORCHESTRATOR] Docking failed — aborting.")
            return

        if not self._do_pick():
            self.get_logger().error("[ORCHESTRATOR] Pick failed.")
            return

        self.get_logger().info("[ORCHESTRATOR] Full pickup sequence complete.")

    # ------------------------------------------------------------------
    # DockToBlock
    # ------------------------------------------------------------------

    def _do_dock(self) -> bool:
        goal = DockToBlock.Goal()
        goal.timeout_sec = self.dock_timeout

        self.get_logger().info(
            f"[ORCHESTRATOR] Sending DockToBlock goal (timeout={self.dock_timeout:.0f}s)…"
        )

        send_future = self._dock_client.send_goal_async(
            goal,
            feedback_callback=self._dock_feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("[ORCHESTRATOR] DockToBlock goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(
                f"[ORCHESTRATOR] Docking succeeded (final error={result.final_error_m*100:.1f} cm)."
            )
            return True
        else:
            self.get_logger().error(f"[ORCHESTRATOR] Docking failed: {result.message}")
            return False

    def _dock_feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        state_names = {0: "SEARCHING", 1: "CANDIDATE", 2: "LOCKED",
                       3: "CONVERGING", 4: "ALIGNED", 5: "WAITING", 6: "ORBITING"}
        self.get_logger().info(
            f"[DOCK] state={state_names.get(fb.state, fb.state)}  "
            f"err={fb.error_m*100:.1f}cm  dx={fb.dx*100:.1f}cm  dy={fb.dy*100:.1f}cm",
            throttle_duration_sec=0.5,
        )

    # ------------------------------------------------------------------
    # Pick
    # ------------------------------------------------------------------

    def _do_pick(self) -> bool:
        colors, count = self._build_pick_colors()

        if count == 0:
            self.get_logger().warning(
                "[ORCHESTRATOR] No blocks in last /best_pickup — sending single-block fallback."
            )
            colors = [1, 0, 0, 0]
            count  = 1

        goal = Pick.Goal()
        goal.colors = colors
        goal.count  = count
        goal.timeout_sec = self.pick_timeout

        self.get_logger().info(
            f"[ORCHESTRATOR] Sending Pick goal: colors={colors}, count={count}"
        )

        send_future = self._pick_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("[ORCHESTRATOR] Pick goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"[ORCHESTRATOR] Pick succeeded: {result.message}")
        else:
            self.get_logger().error(f"[ORCHESTRATOR] Pick failed: {result.message}")

        return result.success

    def _build_pick_colors(self) -> tuple[list[int], int]:
        """
        Convert /best_pickup assignments to the Pick.Goal colors[4] array.

        cup_0 → index 0, cup_1 → index 1, …
        Absent cups stay 0.
        """
        colors = [0, 0, 0, 0]

        if self._last_best_pickup is None:
            return colors, 0

        cup_index = {"cup_0": 0, "cup_1": 1, "cup_2": 2, "cup_3": 3}
        assignments = self._last_best_pickup.get("assignments", [])

        for a in assignments:
            idx = cup_index.get(a.get("cup", ""), -1)
            if idx < 0:
                continue
            colors[idx] = self._color_to_pick_mode(a.get("color", "unknown"))

        count = sum(1 for c in colors if c != 0)
        return colors, count


def main(args=None):
    rclpy.init(args=args)
    node = PickupOrchestrator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
