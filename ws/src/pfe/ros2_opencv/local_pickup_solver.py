#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

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

        # ---------- Team color ----------
        # Set this to "blue" or "yellow"
        self.team_color = "yellow"

        # ---------- Block colors ----------
        # Placeholder map. Replace/update from your own perception node later.
        # Example:
        # self.block_color_map["aruco_47_14"] = "blue"
        self.block_color_map: Dict[str, str] = {}

        # ---------- Matching params ----------
        self.match_radius_m = 0.015
        self.pickup_max_err_m = 0.015

        self.min_useful_matches = 1
        self.pickup_min_matches = 1

        self.tf_timeout = Duration(seconds=0.05)

        # ---------- Fresh block filtering ----------
        self.aruco_last_seen: Dict[str, float] = {}
        self.block_timeout_s = 0.2

        # ---------- Local-mode limits ----------
        self.max_local_dx_m = 0.40
        self.max_local_dy_m = 0.25
        self.max_local_dyaw_deg = 30.0

        # ---------- Yaw search ----------
        self.yaw_min_deg = -20.0
        self.yaw_max_deg = 20.0
        self.yaw_step_deg = 1.5
 
        # ---------- Weighted scoring ---------- ######################## Weighting factors for scoring function. Adjust these to change behavior. Higher w_matches means more aggressive pickups with more matches, higher w_color means more preference for correct color orientation, higher w_error means more penalty for misalignment, etc.
        # Bigger is better
        self.w_matches = 1000.0
        self.w_color = 120.0
        self.w_error = 1.0      # applied to avg error in mm
        self.w_yaw = 2.0        # penalty per degree
        self.w_translation = 80.0  # penalty per meter of base shift

        # ---------- Stability filtering ----------
        self.required_stable_cycles = 3
        self.stable_dx_tol_m = 0.01
        self.stable_dy_tol_m = 0.01
        self.stable_dyaw_tol_deg = 3.0

        self.prev_solution = None
        self.prev_assignment_signature = None
        self.stable_cycles = 0

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(TFMessage, "/tf", self.tf_cb, qos)

        self.timer = self.create_timer(0.2, self.tick)

    def tf_cb(self, msg: TFMessage):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        for t in msg.transforms:
            child = t.child_frame_id
            if child.startswith("aruco"):
                self.aruco_last_seen[child] = now_sec

    def get_fresh_aruco_frames(self) -> List[str]:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        fresh = []
        stale = []

        for frame, last_seen in self.aruco_last_seen.items():
            if (now_sec - last_seen) <= self.block_timeout_s:
                fresh.append(frame)
            else:
                stale.append(frame)

        for frame in stale:
            del self.aruco_last_seen[frame]

        return fresh

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

    def get_block_color(self, frame_name: str) -> str:
        """
        Extract block color from TF name.
        Expected format: aruco_<id>_<index>
        """

        try:
            parts = frame_name.split("_")
            marker_id = int(parts[1])
        except Exception:
            return "unknown"

        # Adjust these if your camera confirms the opposite mapping ##################################### 
        if marker_id == 36:
            return "blue"
        elif marker_id == 47:
            return "yellow"
        else:
            return "unknown"

    def color_value(self, frame_name: str) -> float:  # Color scores for scoring function ###########################
        color = self.get_block_color(frame_name)

        if color == "unknown":
            return 0.0

        # We want the opposite color facing up so we can flip it
        if self.team_color == "yellow":
            if color == "blue":
                return 1.0
            elif color == "yellow":
                return 0.2
        elif self.team_color == "blue":
            if color == "yellow":
                return 1.0
            elif color == "blue":
                return 0.2

        return 0.0

    def score_candidate(
        self,
        rotated_cups: List[Tuple[str, XY]],
        block_list: List[Tuple[str, XY]],
        dx: float,
        dy: float,
        yaw_rad: float,
    ) -> Tuple[int, float, List[Tuple[str, str, float]], float, float]:
        """
        Returns:
            matches,
            weighted_score,
            assignments,
            avg_err_mm,
            color_score
        """
        unmatched_blocks = {name: xy for name, xy in block_list}
        assignments: List[Tuple[str, str, float]] = []
        total_err_m = 0.0
        matches = 0
        color_score = 0.0

        for cup_name, cxy in rotated_cups:
            shifted = XY(cxy.x + dx, cxy.y + dy)

            nearest_block = None
            nearest_dist = None

            for bname, bxy in unmatched_blocks.items():
                d = math.hypot(bxy.x - shifted.x, bxy.y - shifted.y)
                if nearest_dist is None or d < nearest_dist:
                    nearest_dist = d
                    nearest_block = bname

            if nearest_block is not None and nearest_dist is not None and nearest_dist <= self.match_radius_m:
                matches += 1
                total_err_m += nearest_dist
                color_score += self.color_value(nearest_block)
                assignments.append((cup_name, nearest_block, nearest_dist))
                del unmatched_blocks[nearest_block]

        if matches > 0:
            avg_err_mm = (total_err_m / matches) * 1000.0
        else:
            avg_err_mm = 1e9

        yaw_deg = abs(math.degrees(yaw_rad))
        translation_mag = math.hypot(dx, dy)

        weighted_score = (
            self.w_matches * matches
            + self.w_color * color_score
            - self.w_error * avg_err_mm
            - self.w_yaw * yaw_deg
            - self.w_translation * translation_mag
        )

        return matches, weighted_score, assignments, avg_err_mm, color_score

    def compute_best_alignment(
        self,
        cups: Dict[str, XY],
        blocks: Dict[str, XY],
    ) -> Tuple[float, float, float, int, float, List[Tuple[str, str, float]], float, float]:
        best_yaw = 0.0
        best_dx = 0.0
        best_dy = 0.0
        best_matches = -1
        best_score = -1e18
        best_assignments: List[Tuple[str, str, float]] = []
        best_avg_err_mm = 1e9
        best_color_score = 0.0

        block_list = list(blocks.items())

        yaw_deg = self.yaw_min_deg
        while yaw_deg <= self.yaw_max_deg + 1e-9:
            yaw_rad = math.radians(yaw_deg)

            rotated_cups = [
                (name, self.rotate_xy(p, yaw_rad))
                for name, p in cups.items()
            ]

            for _, cup_xy in rotated_cups:
                for _, block_xy in block_list:
                    dx = block_xy.x - cup_xy.x
                    dy = block_xy.y - cup_xy.y

                    if abs(dx) > self.max_local_dx_m:
                        continue
                    if abs(dy) > self.max_local_dy_m:
                        continue

                    matches, score, assignments, avg_err_mm, color_score = self.score_candidate(
                        rotated_cups, block_list, dx, dy, yaw_rad
                    )

                    # First prefer more matches, then weighted score
                    if (
                        matches > best_matches
                        or (matches == best_matches and score > best_score)
                    ):
                        best_yaw = yaw_rad
                        best_dx = dx
                        best_dy = dy
                        best_matches = matches
                        best_score = score
                        best_assignments = assignments
                        best_avg_err_mm = avg_err_mm
                        best_color_score = color_score

        return (
            best_yaw,
            best_dx,
            best_dy,
            best_matches,
            best_score,
            best_assignments,
            best_avg_err_mm,
            best_color_score,
        )

    def solution_is_local(self, dx: float, dy: float, yaw_rad: float) -> bool:
        return (
            abs(dx) <= self.max_local_dx_m
            and abs(dy) <= self.max_local_dy_m
            and abs(math.degrees(yaw_rad)) <= self.max_local_dyaw_deg
        )

    def update_stability(self, dx: float, dy: float, yaw_rad: float):
        yaw_deg = math.degrees(yaw_rad)

        if self.prev_solution is None:
            self.prev_solution = (dx, dy, yaw_deg)
            self.stable_cycles = 1
            return

        prev_dx, prev_dy, prev_yaw_deg = self.prev_solution

        if (
            abs(dx - prev_dx) <= self.stable_dx_tol_m
            and abs(dy - prev_dy) <= self.stable_dy_tol_m
            and abs(yaw_deg - prev_yaw_deg) <= self.stable_dyaw_tol_deg
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
        start_t = time.time()

        # ---------- Cups ----------
        cups: Dict[str, XY] = {}
        for c in self.cup_frames:
            xy = self.lookup_xy(c)
            if xy is not None:
                cups[c] = xy

        if len(cups) != len(self.cup_frames):
            self.get_logger().info(
                f"waiting for cups... got {len(cups)}/{len(self.cup_frames)}"
            )
            self.stable_cycles = 0
            self.prev_solution = None
            self.prev_assignment_signature = None
            return

        # ---------- Fresh blocks ----------
        blocks: Dict[str, XY] = {}
        fresh_frames = self.get_fresh_aruco_frames()

        for b in fresh_frames:
            xy = self.lookup_xy(b)
            if xy is not None:
                blocks[b] = xy

        if len(blocks) == 0:
            self.get_logger().info("waiting for fresh blocks...")
            self.stable_cycles = 0
            self.prev_solution = None
            self.prev_assignment_signature = None
            return

        # ---------- Solve ----------
        (
            best_yaw,
            best_dx,
            best_dy,
            best_matches,
            best_score,
            best_assignments,
            best_avg_err_mm,
            best_color_score,
        ) = self.compute_best_alignment(cups, blocks)

        self.get_logger().info(
            f"debug | fresh_blocks={len(blocks)} | matches={best_matches} | "
            f"score={best_score:.1f} | color_score={best_color_score:.2f} | "
            f"avg_err={best_avg_err_mm:.1f} mm"
        )

        if best_matches < self.min_useful_matches:
            self.get_logger().info(
                f"No worthwhile pickup found. best_matches={best_matches}"
            )
            self.stable_cycles = 0
            self.prev_solution = None
            self.prev_assignment_signature = None
            return

        if not self.solution_is_local(best_dx, best_dy, best_yaw):
            self.get_logger().info(
                f"Best solution outside local window: "
                f"dx={best_dx:+.3f}, dy={best_dy:+.3f}, dyaw={math.degrees(best_yaw):+.1f} deg"
            )
            self.stable_cycles = 0
            self.prev_solution = None
            self.prev_assignment_signature = None
            return

        assignment_signature = tuple(sorted([blk for _, blk, _ in best_assignments]))
        if self.prev_assignment_signature is None:
            self.prev_assignment_signature = assignment_signature
        elif assignment_signature != self.prev_assignment_signature:
            self.stable_cycles = 0
            self.prev_solution = None
            self.prev_assignment_signature = assignment_signature

        self.update_stability(best_dx, best_dy, best_yaw)

        ready = self.is_pickup_ready(best_matches, best_assignments)
        yaw_deg = math.degrees(best_yaw)

        assign_str = ", ".join(
            [f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in best_assignments]
        )

        dt_ms = (time.time() - start_t) * 1000.0

        self.get_logger().info(
            f"BEST ALIGN | dx={best_dx:+.3f} m, dy={best_dy:+.3f} m, "
            f"dyaw={yaw_deg:+.1f} deg | matches={best_matches} | "
            f"score={best_score:.1f} | color_score={best_color_score:.2f} | "
            f"avg_err={best_avg_err_mm:.1f} mm | "
            f"stable={self.stable_cycles}/{self.required_stable_cycles} | "
            f"pickup_ready={ready} | solver_time={dt_ms:.2f} ms | {assign_str}"
        )


def main():
    rclpy.init()
    node = CupBlockAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()