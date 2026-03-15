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
from std_msgs.msg import String
import json
from collections import defaultdict


@dataclass
class XY:
    x: float
    y: float


# Constants
TEAM_COLOR = "yellow"
BLUE_IDS = {36}
YELLOW_IDS = {47}
MATCH_RADIUS_M = 0.015
PICKUP_MAX_ERR_M = 0.015
BLOCK_MEMORY_S = 0.50
MISSING_BLOCK_CYCLES_ALLOWED = 10
MAX_LOCAL_DX_M = 0.40
MAX_LOCAL_DY_M = 0.25
MAX_LOCAL_DYAW_DEG = 95.0
YAW_MIN_DEG = -95.0
YAW_MAX_DEG = 95.0
YAW_STEP_DEG = 1.0
W_MATCHES = 500.0
W_COLOR = 10000.0
W_ERROR = 1.0
W_YAW = 0.3
W_TRANSLATION = 80.0
W_ROW = 400.0
ROW_MAX_PERP_ERROR_M = 0.01
W_CONSECUTIVE = 250.0
REQUIRED_STABLE_CYCLES = 7
STABLE_DX_TOL_M = 0.01
STABLE_DY_TOL_M = 0.01
STABLE_DYAW_TOL_DEG = 3.0
BLOCK_HISTORY_LEN = 10
MOTION_RESET_THRESHOLD_M = 0.03
DEBUG_CANDIDATES = True
DEBUG_TOP_K = 10


class CupBlockAligner(Node):
    def __init__(self):
        super().__init__("cup_block_aligner")

        # ---------- Frames ----------
        self.ref_frame = "base_link"
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]

        # ---------- Team color ----------
        self.team_color = TEAM_COLOR   # or "blue"

        # ---------- Marker ID -> color ----------
        self.BLUE_IDS = BLUE_IDS
        self.YELLOW_IDS = YELLOW_IDS

        # ---------- Matching params ----------
        self.match_radius_m = MATCH_RADIUS_M  # Reduced from 0.020 for tighter matches
        self.pickup_max_err_m = PICKUP_MAX_ERR_M  # Reduced from 0.020 for stricter pickup

        self.min_useful_matches = 2
        self.pickup_min_matches = 2

        self.tf_timeout = Duration(seconds=0.03)
 
        # ---------- Recent visible blocks ----------
        self.block_memory_s = BLOCK_MEMORY_S  # Increased from 0.30 to keep blocks in memory longer
        self.block_last_seen = {}  # Add this (dict to track last seen times)
        self.max_blocks_to_consider = 20  # Add this (limit blocks to consider)

        # ---------- tolerate temporary block loss ----------
        self.missing_block_cycles_allowed = MISSING_BLOCK_CYCLES_ALLOWED  # Increased from 10 for more tolerance (~6s)

        # ---------- Local-mode limits ----------
        self.max_local_dx_m = MAX_LOCAL_DX_M
        self.max_local_dy_m = MAX_LOCAL_DY_M
        self.max_local_dyaw_deg = MAX_LOCAL_DYAW_DEG

        # ---------- Yaw search ----------
        self.yaw_min_deg = YAW_MIN_DEG
        self.yaw_max_deg = YAW_MAX_DEG
        self.yaw_step_deg = YAW_STEP_DEG  # Reduced from 3.0 for finer resolution

        # ---------- Weighted scoring ----------
        self.w_matches = W_MATCHES
        self.w_color = W_COLOR
        self.w_error = W_ERROR
        self.w_yaw = W_YAW
        self.w_translation = W_TRANSLATION

        # ---------- Row bonus ----------
        self.w_row = W_ROW
        self.row_max_perp_error_m = ROW_MAX_PERP_ERROR_M

        # ---------- Consecutive cups bonus ----------
        self.w_consecutive = W_CONSECUTIVE

        # ---------- Stability filtering ----------
        self.required_stable_cycles = REQUIRED_STABLE_CYCLES  # Reduced from 5 for faster locking
        self.stable_dx_tol_m = STABLE_DX_TOL_M
        self.stable_dy_tol_m = STABLE_DY_TOL_M
        self.stable_dyaw_tol_deg = STABLE_DYAW_TOL_DEG

        self.prev_solution = None
        self.prev_assignment_signature = None
        self.stable_cycles = 0

        # ---------- Block stability tracking + smoothing ----------
        self.block_history: Dict[str, List[XY]] = {}
        self.block_history_len = BLOCK_HISTORY_LEN
        self.use_smoothed_blocks = True
        self.motion_reset_threshold_m = MOTION_RESET_THRESHOLD_M

        # ---------- Debug ----------
        self.debug_candidates = DEBUG_CANDIDATES
        self.debug_top_k = DEBUG_TOP_K
        self.debug_tf_children = False
        self.debug_geometry = False
        self.debug_block_stability = False

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(TFMessage, "/tf", self.tf_cb, qos)

        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info("✅ CupBlockAligner using block_* frames.")

        self.pickup_goal_pub = self.create_publisher(String, "pickup_goal", 10)
        self.block_queue_pub = self.create_publisher(String, "block_queue_cmd", 10)
        self.pickup_state_sub = self.create_subscription(
            String, "pickup_state", self.pickup_state_cb, 10
        )

        self.locked = False
        self.locked_signature = None
        self.locked_assignments = None  # Add this
        self.pickup_state = "idle"  # idle / moving / arrived / picking / done
        self.pickup_timer = None

    def tf_cb(self, msg: TFMessage):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        for t in msg.transforms:
            child = t.child_frame_id
            if self.debug_tf_children:
                self.get_logger().info(f"tf child seen: {child}")
            if child.lower().startswith("block_"):
                self.block_last_seen[child] = now_sec

    def get_recent_block_frames(self) -> List[str]:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        recent = []
        stale = []

        for frame, last_seen in self.block_last_seen.items():
            if (now_sec - last_seen) <= self.block_memory_s:
                recent.append(frame)
            else:
                stale.append(frame)

        for frame in stale:
            del self.block_last_seen[frame]
            if frame in self.block_history:
                del self.block_history[frame]

        recent.sort()
        return recent[:self.max_blocks_to_consider]

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

    def debug_pairwise_spacing(self, label: str, items: Dict[str, XY], sort_axis: str = "y"):
        if len(items) == 0:
            return

        if sort_axis == "x":
            ordered = sorted(items.items(), key=lambda item: item[1].x)
        else:
            ordered = sorted(items.items(), key=lambda item: item[1].y)

        self.get_logger().info(f"----- {label} ORDERED BY {sort_axis.upper()} -----")
        for name, xy in ordered:
            self.get_logger().info(f"{name}: x={xy.x:+.3f}, y={xy.y:+.3f}")

        if len(ordered) >= 2:
            self.get_logger().info(f"----- {label} SPACING -----")
            for i in range(len(ordered) - 1):
                n1, p1 = ordered[i]
                n2, p2 = ordered[i + 1]
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                d = math.hypot(dx, dy)
                self.get_logger().info(
                    f"{n1} -> {n2}: dx={dx:+.3f}, dy={dy:+.3f}, dist={d*1000:.1f} mm"
                )

    def compute_row_bonus(self, assignments: List[Tuple[str, str, float]], blocks: Dict[str, XY]) -> float:
        if len(assignments) < 2:
            return 0.0

        matched_points = []
        for _, blk_name, _ in assignments:
            if blk_name in blocks:
                matched_points.append(blocks[blk_name])

        if len(matched_points) < 2:
            return 0.0

        best_avg_perp = float("inf")

        for i in range(len(matched_points)):
            for j in range(i + 1, len(matched_points)):
                p1 = matched_points[i]
                p2 = matched_points[j]

                dx = p2.x - p1.x
                dy = p2.y - p1.y
                norm = math.hypot(dx, dy)
                if norm < 1e-9:
                    continue

                ux = dx / norm
                uy = dy / norm

                perp_errors = []
                for p in matched_points:
                    vx = p.x - p1.x
                    vy = p.y - p1.y
                    perp = abs(vx * uy - vy * ux)
                    perp_errors.append(perp)

                avg_perp = sum(perp_errors) / len(perp_errors)
                if avg_perp < best_avg_perp:
                    best_avg_perp = avg_perp

        if best_avg_perp == float("inf"):
            return 0.0

        if best_avg_perp >= self.row_max_perp_error_m:
            return 0.0

        return self.w_row * (1.0 - best_avg_perp / self.row_max_perp_error_m)

    def compute_consecutive_bonus(self, assignments: List[Tuple[str, str, float]]) -> float:
        if len(assignments) < 2:
            return 0.0

        cup_indices = []
        for cup_name, _, _ in assignments:
            try:
                idx = int(cup_name.split("_")[1])
                cup_indices.append(idx)
            except Exception:
                continue

        if len(cup_indices) < 2:
            return 0.0

        cup_indices = sorted(set(cup_indices))

        longest_run = 1
        current_run = 1
        for i in range(1, len(cup_indices)):
            if cup_indices[i] == cup_indices[i - 1] + 1:
                current_run += 1
                longest_run = max(longest_run, current_run)
            else:
                current_run = 1

        if longest_run < 2:
            return 0.0

        return self.w_consecutive * (longest_run - 1)

    def get_block_color(self, frame_name: str) -> str:
        try:
            parts = frame_name.split("_")
            marker_id = int(parts[1])
        except Exception:
            return "unknown"

        if marker_id in self.BLUE_IDS:
            return "blue"
        elif marker_id in self.YELLOW_IDS:
            return "yellow"
        else:
            return "unknown"

    def color_value(self, frame_name: str) -> float:
        color = self.get_block_color(frame_name)

        if color == "unknown":
            return 0.0

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

    def update_block_history(self, blocks: Dict[str, XY]):
        for name, xy in blocks.items():
            if name not in self.block_history:
                self.block_history[name] = [xy]
                continue

            last = self.block_history[name][-1]
            motion = math.hypot(xy.x - last.x, xy.y - last.y)

            if motion > self.motion_reset_threshold_m:
                self.block_history[name] = [xy]
            else:
                self.block_history[name].append(xy)
                if len(self.block_history[name]) > self.block_history_len:
                    self.block_history[name].pop(0)

    def print_block_stability(self):
        if not self.debug_block_stability:
            return

        for name, history in self.block_history.items():
            if len(history) < 2:
                continue

            xs = [p.x for p in history]
            ys = [p.y for p in history]

            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)

            std_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs) / len(xs))
            std_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys) / len(ys))

            self.get_logger().info(
                f"{name} | mean=({mean_x:.3f},{mean_y:.3f}) | "
                f"std=({std_x*1000:.1f}mm,{std_y*1000:.1f}mm)"
            )

    def get_smoothed_blocks(self, raw_blocks: Dict[str, XY]) -> Dict[str, XY]:
        if not self.use_smoothed_blocks:
            return raw_blocks

        smoothed = {}

        for name in raw_blocks:
            history = self.block_history.get(name, [])
            if len(history) == 0:
                smoothed[name] = raw_blocks[name]
                continue

            xs = [p.x for p in history]
            ys = [p.y for p in history]

            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)

            smoothed[name] = XY(mean_x, mean_y)

        return smoothed

    def score_candidate(
        self,
        rotated_cups: List[Tuple[str, XY]],
        block_list: List[Tuple[str, XY]],
        dx: float,
        dy: float,
        yaw_rad: float,
        blocks_dict: Dict[str, XY],
    ) -> Tuple[int, float, List[Tuple[str, str, float]], float, float]:
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
        row_bonus = self.compute_row_bonus(assignments, blocks_dict)
        consecutive_bonus = self.compute_consecutive_bonus(assignments)

        weighted_score = (
            self.w_matches * matches
            + self.w_color * color_score
            + row_bonus
            + consecutive_bonus
            - self.w_error * avg_err_mm
            - self.w_yaw * yaw_deg
            - self.w_translation * translation_mag
        )

        return matches, weighted_score, assignments, avg_err_mm, color_score
    
    def generate_candidates(self, cups: Dict[str, XY], blocks: Dict[str, XY]) -> List[Dict]:
        """Generate all possible alignment candidates."""
        candidates = []
        block_list = list(blocks.items())
        
        yaw_deg = YAW_MIN_DEG
        while yaw_deg <= YAW_MAX_DEG + 1e-9:
            yaw_rad = math.radians(yaw_deg)
            rotated_cups = [(name, self.rotate_xy(p, yaw_rad)) for name, p in cups.items()]
            
            for _, cup_xy in rotated_cups:
                for _, block_xy in block_list:
                    dx = block_xy.x - cup_xy.x
                    dy = block_xy.y - cup_xy.y
                    if abs(dx) > MAX_LOCAL_DX_M or abs(dy) > MAX_LOCAL_DY_M:
                        continue
                    
                    matches, score, assignments, avg_err_mm, color_score = self.score_candidate(
                        rotated_cups, block_list, dx, dy, yaw_rad, blocks
                    )
                    if matches >= self.min_useful_matches:
                        candidates.append({
                            "matches": matches, "score": score, "yaw_deg": yaw_deg,
                            "dx": dx, "dy": dy, "avg_err_mm": avg_err_mm,
                            "color_score": color_score, "assignments": assignments,
                        })
            yaw_deg += YAW_STEP_DEG
        return candidates

    def select_best_candidate(self, candidates: List[Dict]) -> Dict:
        """Select the best candidate from the list."""
        if not candidates:
            return {}
        return max(candidates, key=lambda c: (c["matches"], c["score"]))

    def deduplicate_candidates(self, candidates: List[Dict]) -> List[Dict]:
        """Deduplicate by assignment set."""
        best_per_assignment = defaultdict(lambda: {"score": -1e18, "cand": None})
        for cand in candidates:
            assign_sig = tuple(sorted((cup, blk) for cup, blk, _ in cand["assignments"]))
            if cand["score"] > best_per_assignment[assign_sig]["score"]:
                best_per_assignment[assign_sig] = {"score": cand["score"], "cand": cand}
        return [v["cand"] for v in best_per_assignment.values() if v["cand"] is not None]

    def compute_best_alignment(self, cups: Dict[str, XY], blocks: Dict[str, XY]) -> Tuple[...]:
        candidates = self.generate_candidates(cups, blocks)
        candidates = self.deduplicate_candidates(candidates)
        candidates.sort(key=lambda c: (c["matches"], c["score"]), reverse=True)
        
        best_cand = self.select_best_candidate(candidates)
        if not best_cand:
            return (0.0, 0.0, 0.0, 0, -1e18, [], 1e9, 0.0)
        
        # Debug logging (only if enabled)
        if DEBUG_CANDIDATES and candidates:
            self.log_candidates(candidates)
        
        return (
            math.radians(best_cand["yaw_deg"]), best_cand["dx"], best_cand["dy"],
            best_cand["matches"], best_cand["score"], best_cand["assignments"],
            best_cand["avg_err_mm"], best_cand["color_score"]
        )

    def log_candidates(self, candidates: List[Dict]):
        """Log top candidates."""
        four_match = [c for c in candidates if c["matches"] == 4]
        self.get_logger().info(f"debug | num_4_match_candidates={len(four_match)}")
        self.get_logger().info("===== TOP CANDIDATES =====")
        for i, cand in enumerate(candidates):
            assign_str = ", ".join([f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in cand["assignments"]])
            self.get_logger().info(f"[{i}] matches={cand['matches']} | score={cand['score']:.1f} | ... | {assign_str}")
        self.get_logger().info("==========================")


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
    
    def pickup_state_cb(self, msg: String):
        s = msg.data.strip().lower()
        if s == "arrived":
            self.get_logger().info("pickup_state: arrived")
            self.pickup_state = "arrived"
            # Publish the block queue now that robot is at pickup location
            if self.locked and self.locked_assignments is not None:
                self.publish_block_queue(self.locked_assignments)
                self.get_logger().info("PUBLISHED block queue after arrived")
        elif s == "done":
            self.get_logger().info("pickup_state: done")
            self.pickup_state = "done"
            self.locked = False
            self.locked_signature = None
            self.locked_assignments = None  # Clear
        elif s == "reset":
            self.get_logger().info("pickup_state: reset")
            self.locked = False
            self.pickup_state = "idle"
            self.locked_signature = None
            self.locked_assignments = None

    
    def publish_block_queue(self, assignments: List[Tuple[str, str, float]]):
        # Cup order: 3 first, then 2, 1, 0 (as you described)
        ordered = sorted(
            assignments,
            key=lambda a: int(a[0].split("_")[1]),
            reverse=True,
        )
        cmd_parts = []
        for cup, blk, _ in ordered:
            color = "Y" if self.get_block_color(blk) == "yellow" else "B"
            idx = cup.split("_")[1]
            cmd_parts.append(f"C{idx}:{color}")
        msg = String()
        msg.data = ",".join(cmd_parts)
        self.block_queue_pub.publish(msg)


    def publish_pickup_goal(self, dx, dy, yaw_deg, assignments):
        plan = {
            "dx": dx,
            "dy": dy,
            "yaw_deg": yaw_deg,
            "assignments": [
                {
                    "cup": cup,
                    "block": blk,
                    "color": self.get_block_color(blk)
                }
                for cup, blk, _ in assignments
            ],
        }
        msg = String()
        msg.data = json.dumps(plan)
        self.pickup_goal_pub.publish(msg)

    def tick(self):
        try:
            start_t = time.time()

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

            raw_blocks: Dict[str, XY] = {}
            recent_frames = self.get_recent_block_frames()

            for b in recent_frames:
                xy = self.lookup_xy(b)
                if xy is not None:
                    raw_blocks[b] = xy

            self.update_block_history(raw_blocks)
            self.print_block_stability()
            blocks = self.get_smoothed_blocks(raw_blocks)

            self.get_logger().debug(
                f"debug | recent_block_frames={len(recent_frames)} | valid_blocks={len(blocks)}"
            )

            if self.locked:
                # Skip debug output while locked to reduce log spam
                return

            if len(blocks) == 0:
                self.get_logger().info("waiting for fresh blocks...")
                self.stable_cycles = 0
                self.prev_solution = None
                self.prev_assignment_signature = None
                return

            if self.debug_geometry:
                self.get_logger().info("===== CUP POSITIONS =====")
                for name, xy in cups.items():
                    self.get_logger().info(f"{name}: x={xy.x:+.3f}, y={xy.y:+.3f}")

                self.get_logger().info("===== BLOCK POSITIONS =====")
                for name, xy in blocks.items():
                    color = self.get_block_color(name)
                    self.get_logger().info(f"{name} ({color}): x={xy.x:+.3f}, y={xy.y:+.3f}")

                self.debug_pairwise_spacing("CUPS", cups, sort_axis="y")
                self.debug_pairwise_spacing("BLOCKS", blocks, sort_axis="y")

            self.get_logger().info("debug | entering compute_best_alignment()")
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
            self.get_logger().info("debug | compute_best_alignment() returned")

            self.get_logger().info(
                f"debug | matches={best_matches} | "
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

            # Compute stability / readiness before locking
            yaw_deg = math.degrees(best_yaw)

            # create a stable signature for the current assignment set
            assignment_signature = tuple(sorted([blk for _, blk, _ in best_assignments]))

            # if the assignment set changed, reset stability
            if assignment_signature != self.prev_assignment_signature:
                self.stable_cycles = 1
                self.prev_solution = (best_dx, best_dy, yaw_deg)
                self.prev_assignment_signature = assignment_signature
            else:
                self.update_stability(best_dx, best_dy, best_yaw)

            ready = self.is_pickup_ready(best_matches, best_assignments)

            # when ready and not currently locked, send a single pickup command
            if ready and not self.locked:
                self.locked = True
                self.locked_signature = assignment_signature
                self.locked_assignments = best_assignments  # Add this
                self.pickup_state = "moving"
                self.publish_pickup_goal(best_dx, best_dy, yaw_deg, best_assignments)
                self.get_logger().info("PUBLISHED pickup goal, locking until done")

                # Log the locked assignments
                locked_assign_str = ", ".join(
                    [f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in best_assignments]
                )
                self.get_logger().info(f"Locked solution blocks: {locked_assign_str}")

            # while locked, do not change solution unless unlocked
            if self.locked:
                # clear debug output but keep solver running (optional)
                return

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

        except Exception as e:
            self.get_logger().error(f"tick() crashed: {repr(e)}")


def main():
    rclpy.init()
    node = CupBlockAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()