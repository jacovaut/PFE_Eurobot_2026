#!/usr/bin/env python3
"""
pickup_orchestrator.py
----------------------
Chains the DockToBlock, Pickup, and Flip actions:
  1. Waits for DockToBlock to succeed
  2. Reads the last /best_pickup to extract block colors
    3. Sends a Pickup goal for all assigned cups
    4. Sends a Flip goal only for cups that need flipping

Run with:
  ros2 run local_camera pickup_orchestrator
"""

import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from custom_msgs.action import DockToBlock, Flip, Pickup

# Color string from solver → Flip.Goal colors array value
# 0 = skip, 1 = blue flip sequence, 2 = yellow flip sequence
_FLIP_COLOR_TO_INT = {
    "blue": 1,
    "yellow": 2,
    "unknown": 0,
}


class PickupOrchestrator(Node):
    def __init__(self):
        super().__init__("pickup_orchestrator")

        self.declare_parameter("dock_timeout_sec", 30.0)
        self.declare_parameter("pickup_timeout_sec", 60.0)
        self.declare_parameter("flip_timeout_sec", 30.0)

        self.dock_timeout = float(self.get_parameter("dock_timeout_sec").value)
        self.pickup_timeout = float(self.get_parameter("pickup_timeout_sec").value)
        self.flip_timeout = float(self.get_parameter("flip_timeout_sec").value)

        # Last known assignment from the solver
        self._last_best_pickup: dict | None = None

        self.create_subscription(
            String,
            "/best_pickup",
            self._best_pickup_cb,
            10,
        )

        self._dock_client = ActionClient(self, DockToBlock, "dock_to_block")
        self._pickup_client = ActionClient(self, Pickup, "pickup")
        self._flip_client = ActionClient(self, Flip, "flip")

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
        self._pickup_client.wait_for_server()
        self._flip_client.wait_for_server()
        self.get_logger().info("[ORCHESTRATOR] Action servers ready.")

        if not self._do_dock():
            self.get_logger().error("[ORCHESTRATOR] Docking failed — aborting.")
            return

        if not self._do_pickup():
            self.get_logger().error("[ORCHESTRATOR] Pickup failed.")
            return

        if not self._do_flip():
            self.get_logger().error("[ORCHESTRATOR] Flip failed.")
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
    # Pickup / Flip
    # ------------------------------------------------------------------

    def _do_pickup(self) -> bool:
        slots, count = self._build_pickup_slots()

        if count == 0:
            self.get_logger().warning(
                "[ORCHESTRATOR] No blocks in last /best_pickup — sending single-slot pickup fallback."
            )
            slots = [1, 0, 0, 0]
            count  = 1

        goal = Pickup.Goal()
        goal.slots = slots
        goal.count  = count
        goal.timeout_sec = self.pickup_timeout

        self.get_logger().info(
            f"[ORCHESTRATOR] Sending Pickup goal: slots={slots}, count={count}"
        )

        send_future = self._pickup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("[ORCHESTRATOR] Pickup goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"[ORCHESTRATOR] Pickup succeeded: {result.message}")
        else:
            self.get_logger().error(f"[ORCHESTRATOR] Pickup failed: {result.message}")

        return result.success

    def _do_flip(self) -> bool:
        colors, count = self._build_flip_colors()

        if count == 0:
            self.get_logger().info("[ORCHESTRATOR] No flip required for this pickup.")
            return True

        goal = Flip.Goal()
        goal.colors = colors
        goal.count = count
        goal.timeout_sec = self.flip_timeout

        self.get_logger().info(
            f"[ORCHESTRATOR] Sending Flip goal: colors={colors}, count={count}"
        )

        send_future = self._flip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("[ORCHESTRATOR] Flip goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"[ORCHESTRATOR] Flip succeeded: {result.message}")
        else:
            self.get_logger().error(f"[ORCHESTRATOR] Flip failed: {result.message}")

        return result.success

    def _build_pickup_slots(self) -> tuple[list[int], int]:
        slots = [0, 0, 0, 0]

        if self._last_best_pickup is None:
            return slots, 0

        cup_index = {"cup_0": 0, "cup_1": 1, "cup_2": 2, "cup_3": 3}
        assignments = self._last_best_pickup.get("assignments", [])

        for a in assignments:
            idx = cup_index.get(a.get("cup", ""), -1)
            if idx < 0:
                continue
            slots[idx] = 1

        count = sum(slots)
        return slots, count

    def _build_flip_colors(self) -> tuple[list[int], int]:
        colors = [0, 0, 0, 0]

        if self._last_best_pickup is None:
            return colors, 0

        cup_index = {"cup_0": 0, "cup_1": 1, "cup_2": 2, "cup_3": 3}
        assignments = self._last_best_pickup.get("assignments", [])

        for a in assignments:
            idx = cup_index.get(a.get("cup", ""), -1)
            if idx < 0:
                continue
            if a.get("color") == "yellow":
                colors[idx] = _FLIP_COLOR_TO_INT.get(a.get("color", "unknown"), 0)

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
