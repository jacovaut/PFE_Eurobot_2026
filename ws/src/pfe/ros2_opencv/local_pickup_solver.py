#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, List, Set, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage


@dataclass
class XY:
    x: float
    y: float


class CupBlockAligner(Node):
    def __init__(self):
        super().__init__("cup_block_aligner")

        # ---------- Frames ----------
        self.ref_frame = "base_link"
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]

        # ---------- Matching params ----------
        # 2 mm tolerance for testing
        self.match_radius_m = 0.03 

        # Minimum number of matched cups required to care about the solution
        self.min_useful_matches = 2

        # TF lookup timeout
        self.tf_timeout = Duration(seconds=0.05)

        # ---------- Local-mode limits ----------
        # Reject solutions outside this local correction window
        self.max_local_dx_m = 0.5
        self.max_local_dy_m = 0.5
        self.max_local_dyaw_deg = 25.0

        # ---------- Yaw search ----------
        self.yaw_min_deg = -35.0
        self.yaw_max_deg = 35.0
        self.yaw_step_deg = 2.0

        # Slight preference for smaller yaw changes
        self.yaw_penalty = 0.002

        # ---------- Stability filtering ----------
        self.required_stable_cycles = 3
        self.stable_dx_tol_m = 0.01       # 10 mm
        self.stable_dy_tol_m = 0.01       # 10 mm
        self.stable_dyaw_tol_deg = 3.0    # 3 deg

        self.prev_solution = None
        self.stable_cycles = 0

        # ---------- Pickup-ready thresholds ----------
        self.pickup_max_err_m = 0.03     # each matched cup must be within 2 mm
        self.pickup_min_matches = 2

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.aruco_frames: Set[str] = set()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(TFMessage, "/tf", self.tf_cb, qos)

        self.timer = self.create_timer(0.2, self.tick)

    def tf_cb(self, msg: TFMessage):
        for t in msg.transforms:
            child = t.child_frame_id
            if child.startswith("aruco"):
                self.aruco_frames.add(child)

    def lookup_xy(self, target_frame: str) -> Optional[XY]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.ref_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=self.tf_timeout,
            )
            tr = tf.transform.translation
            return XY(tr.x, tr.y)
        except Exception:
            return None

    @staticmethod
    def rotate_xy(p: XY, yaw_rad: float) -> XY:
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return XY(
            x=c * p.x - s * p.y,
            y=s * p.x + c * p.y,
        )

    def score_candidate(
        self,
        rotated_cups: Dict[str, XY],
        blocks: Dict[str, XY],
        dx: float,
        dy: float,
        yaw_rad: float,
    ) -> Tuple[int, float, List[Tuple[str, str, float]]]:
        unmatched_blocks = set(blocks.keys())
        assignments: List[Tuple[str, str, float]] = []
        total_err = 0.0
        matches = 0

        for cup_name, cxy in rotated_cups.items():
            shifted = XY(cxy.x + dx, cxy.y + dy)

            nearest_block = None
            nearest_dist = None

            for bname in unmatched_blocks:
                bxy = blocks[bname]
                d = math.hypot(bxy.x - shifted.x, bxy.y - shifted.y)
                if nearest_dist is None or d < nearest_dist:
                    nearest_dist = d
                    nearest_block = bname

            if nearest_block is not None and nearest_dist is not None and nearest_dist <= self.match_radius_m:
                matches += 1
                total_err += nearest_dist
                assignments.append((cup_name, nearest_block, nearest_dist))
                unmatched_blocks.remove(nearest_block)

        cost = total_err + self.yaw_penalty * abs(yaw_rad)
        return matches, cost, assignments

    def compute_best_alignment(
        self,
        cups: Dict[str, XY],
        blocks: Dict[str, XY],
    ) -> Tuple[float, float, float, int, float, List[Tuple[str, str, float]]]:
        best_yaw = 0.0
        best_dx = 0.0
        best_dy = 0.0
        best_matches = -1
        best_cost = float("inf")
        best_assignments: List[Tuple[str, str, float]] = []

        yaw_deg = self.yaw_min_deg
        while yaw_deg <= self.yaw_max_deg + 1e-9:
            yaw_rad = math.radians(yaw_deg)

            rotated_cups = {
                name: self.rotate_xy(p, yaw_rad)
                for name, p in cups.items()
            }

            for _, cup_xy in rotated_cups.items():
                for _, block_xy in blocks.items():
                    dx = block_xy.x - cup_xy.x
                    dy = block_xy.y - cup_xy.y

                    matches, cost, assignments = self.score_candidate(
                        rotated_cups, blocks, dx, dy, yaw_rad
                    )

                    if matches > best_matches or (matches == best_matches and cost < best_cost):
                        best_yaw = yaw_rad
                        best_dx = dx
                        best_dy = dy
                        best_matches = matches
                        best_cost = cost
                        best_assignments = assignments

            yaw_deg += self.yaw_step_deg

        return best_yaw, best_dx, best_dy, best_matches, best_cost, best_assignments

    def solution_is_local(self, dx: float, dy: float, yaw_rad: float) -> bool:
        return (
            abs(dx) <= self.max_local_dx_m and
            abs(dy) <= self.max_local_dy_m and
            abs(math.degrees(yaw_rad)) <= self.max_local_dyaw_deg
        )

    def update_stability(self, dx: float, dy: float, yaw_rad: float):
        yaw_deg = math.degrees(yaw_rad)

        if self.prev_solution is None:
            self.prev_solution = (dx, dy, yaw_deg)
            self.stable_cycles = 1
            return

        prev_dx, prev_dy, prev_yaw_deg = self.prev_solution

        if (
            abs(dx - prev_dx) <= self.stable_dx_tol_m and
            abs(dy - prev_dy) <= self.stable_dy_tol_m and
            abs(yaw_deg - prev_yaw_deg) <= self.stable_dyaw_tol_deg
        ):
            self.stable_cycles += 1
        else:
            self.stable_cycles = 1

        self.prev_solution = (dx, dy, yaw_deg)

    def is_pickup_ready(self, matches: int, assignments: List[Tuple[str, str, float]]) -> bool:
        if matches < self.pickup_min_matches:
            return False

        if self.stable_cycles < self.required_stable_cycles:
            return False

        for _, _, dist in assignments:
            if dist > self.pickup_max_err_m:
                return False

        return True

    def tick(self):
        cups: Dict[str, XY] = {}
        for c in self.cup_frames:
            xy = self.lookup_xy(c)
            if xy is not None:
                cups[c] = xy

        blocks: Dict[str, XY] = {}
        for b in list(self.aruco_frames):
            xy = self.lookup_xy(b)
            if xy is not None:
                blocks[b] = xy

        if len(cups) != len(self.cup_frames):
            self.get_logger().info(
                f"waiting for cups... got {len(cups)}/{len(self.cup_frames)}"
            )
            return

        if len(blocks) == 0:
            self.get_logger().info("waiting for blocks...")
            return

        best_yaw, best_dx, best_dy, best_matches, best_cost, best_assignments = \
            self.compute_best_alignment(cups, blocks)

        if best_matches < self.min_useful_matches:
            self.get_logger().info(
                f"No worthwhile pickup found. best_matches={best_matches}"
            )
            self.stable_cycles = 0
            self.prev_solution = None
            return

        if not self.solution_is_local(best_dx, best_dy, best_yaw):
            self.get_logger().info(
                f"Best solution outside local window: "
                f"dx={best_dx:+.3f}, dy={best_dy:+.3f}, dyaw={math.degrees(best_yaw):+.1f} deg"
            )
            self.stable_cycles = 0
            self.prev_solution = None
            return

        self.update_stability(best_dx, best_dy, best_yaw)

        yaw_deg = math.degrees(best_yaw)
        assign_str = ", ".join(
            [f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in best_assignments]
        )

        ready = self.is_pickup_ready(best_matches, best_assignments)

        self.get_logger().info(
            f"BEST ALIGN | dx={best_dx:+.3f} m, dy={best_dy:+.3f} m, "
            f"dyaw={yaw_deg:+.1f} deg | matches={best_matches} | "
            f"stable={self.stable_cycles}/{self.required_stable_cycles} | "
            f"pickup_ready={ready} | {assign_str}"
        )


def main():
    rclpy.init()
    node = CupBlockAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()