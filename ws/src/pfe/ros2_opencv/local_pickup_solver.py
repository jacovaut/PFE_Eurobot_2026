#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

import rclpy
from rclpy.node import Node
import tf2_ros
from tf_transformations import euler_from_quaternion


# ----------------------------
# Data models
# ----------------------------

@dataclass
class BlockObs:
    uid: int        # unique instance token (aruco_id*1000 + index)
    aruco_id: int   # actual marker id (e.g., 36, 47)
    x: float        # in base_link (m)
    y: float        # in base_link (m)
    yaw: float      # block yaw in base_link (rad)


@dataclass
class PickupPlan:
    theta_arm: float                 # desired arm axis yaw in base_link (rad)
    u0: float                        # center shift along arm axis (m)
    matches: Dict[int, int]          # cup_index -> uid
    score: int                       # number of matched cups
    mean_abs_v: float                # lateral quality metric (smaller = better)


# ----------------------------
# Math helpers
# ----------------------------

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def angle_dist(a: float, b: float) -> float:
    return abs(wrap_pi(a - b))


def is_perpendicular(block_yaw: float, theta_arm: float, tol_rad: float) -> bool:
    """
    Block yaw should be theta_arm ± 90° modulo pi.
    Equivalent: distance to (theta_arm + pi/2) modulo pi is small.
    """
    target = wrap_pi(theta_arm + math.pi / 2.0)
    d = angle_dist(block_yaw, target)
    d = min(d, abs(d - math.pi))
    return d <= tol_rad


def pca_direction(points: List[Tuple[float, float]]) -> Optional[float]:
    if len(points) < 2:
        return None
    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)

    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)

    if abs(sxy) < 1e-12 and abs(sxx - syy) < 1e-12:
        return 0.0
    return 0.5 * math.atan2(2.0 * sxy, (sxx - syy))


def compute_best_pickup_plan(
    blocks: List[BlockObs],
    cups_spacing_m: float = 0.05,
    tol_perp_m: float = 0.02,
    tol_along_m: float = 0.02,
    tol_angle_deg: float = 15.0,
) -> Optional[PickupPlan]:
    """
    Finds the best (theta_arm, u0) placing 4 cups on up to 4 blocks.

    Arm axis is along +x in the robot when theta_arm == 0 (base_link frame).
    """
    if not blocks:
        return None

    cups = [-1.5 * cups_spacing_m, -0.5 * cups_spacing_m,
             0.5 * cups_spacing_m,  1.5 * cups_spacing_m]
    tol_angle = math.radians(tol_angle_deg)

    pts = [(b.x, b.y) for b in blocks]
    base_theta = pca_direction(pts)
    if base_theta is None:
        b0 = min(blocks, key=lambda b: b.x * b.x + b.y * b.y)
        base_theta = math.atan2(b0.y, b0.x)

    candidates = [
        base_theta,
        wrap_pi(base_theta + math.pi),
        wrap_pi(base_theta + math.radians(10)),
        wrap_pi(base_theta - math.radians(10)),
    ]

    best: Optional[PickupPlan] = None

    for theta in candidates:
        c = math.cos(theta)
        s = math.sin(theta)

        arm_blocks = []
        for b in blocks:
            u = c * b.x + s * b.y
            v = -s * b.x + c * b.y

            if abs(v) > tol_perp_m:
                continue
            if not is_perpendicular(b.yaw, theta, tol_angle):
                continue

            # IMPORTANT: use uid here (unique per instance)
            arm_blocks.append((b.uid, u, v))

        if not arm_blocks:
            continue

        for (uid, u, _v) in arm_blocks:
            for cup_u in cups:
                u0 = u - cup_u

                unused = {ab[0] for ab in arm_blocks}
                matches: Dict[int, int] = {}
                abs_v_list = []

                for cup_idx, cup_off in enumerate(cups):
                    target_u = u0 + cup_off

                    best_blk = None
                    best_du = None
                    best_v = None

                    for (uid2, u2, v2) in arm_blocks:
                        if uid2 not in unused:
                            continue
                        du = abs(u2 - target_u)
                        if du <= tol_along_m and (best_du is None or du < best_du):
                            best_du = du
                            best_blk = uid2
                            best_v = v2

                    if best_blk is not None:
                        matches[cup_idx] = best_blk
                        unused.remove(best_blk)
                        abs_v_list.append(abs(best_v))

                score = len(matches)
                if score == 0:
                    continue

                mean_abs_v = sum(abs_v_list) / len(abs_v_list) if abs_v_list else 1e9

                plan = PickupPlan(theta_arm=theta, u0=u0, matches=matches, score=score, mean_abs_v=mean_abs_v)

                if (best is None or
                    plan.score > best.score or
                    (plan.score == best.score and plan.mean_abs_v < best.mean_abs_v)):
                    best = plan

    return best


# ----------------------------
# ROS2 Node
# ----------------------------

class LocalPickupSolver(Node):
    def __init__(self):
        super().__init__('local_pickup_solver')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frames
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.aruco_prefix = self.declare_parameter('aruco_prefix', 'aruco_').value

        # Which ArUco IDs to consider
        self.aruco_ids = self.declare_parameter('aruco_ids', [36, 47]).value

        # Solver params
        self.cups_spacing_m = float(self.declare_parameter('cups_spacing_m', 0.05).value)
        self.tol_perp_m = float(self.declare_parameter('tol_perp_m', 0.02).value)
        self.tol_along_m = float(self.declare_parameter('tol_along_m', 0.02).value)
        self.tol_angle_deg = float(self.declare_parameter('tol_angle_deg', 15.0).value)

        # Filter window
        self.max_dist_m = float(self.declare_parameter('max_dist_m', 1.5).value)
        self.min_x_m = float(self.declare_parameter('min_x_m', 0.05).value)

        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz

        self.get_logger().info("✅ LocalPickupSolver running. Auto-discovering aruco_<id>_<idx> TF frames...")

    def _discover_aruco_frames(self):
        """
        Returns list of tuples: (frame_name, aruco_id, idx)
        Accepts frames like:
          aruco_47_5
        Also accepts aruco_47 (idx=0) if it exists.
        """
        frames_yaml = self.tf_buffer.all_frames_as_yaml()

        found = []
        for line in frames_yaml.splitlines():
            line = line.strip()
            if not line.endswith(":"):
                continue
            frame = line[:-1]  # remove ':'

            if not frame.startswith(self.aruco_prefix):
                continue

            rest = frame[len(self.aruco_prefix):]
            parts = rest.split("_")

            try:
                if len(parts) == 1:
                    mid = int(parts[0])
                    idx = 0
                elif len(parts) == 2:
                    mid = int(parts[0])
                    idx = int(parts[1])
                else:
                    continue
            except ValueError:
                continue

            if mid not in self.aruco_ids:
                continue

            found.append((frame, mid, idx))

        return found

    def tick(self):
        blocks: List[BlockObs] = []

        candidates = self._discover_aruco_frames()

        for (target_frame, mid, idx) in candidates:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    target_frame,
                    rclpy.time.Time()
                )

                x = float(t.transform.translation.x)
                y = float(t.transform.translation.y)

                if x < self.min_x_m:
                    continue
                if math.hypot(x, y) > self.max_dist_m:
                    continue

                q = t.transform.rotation
                _roll, _pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                uid = int(mid) * 1000 + int(idx)
                blocks.append(BlockObs(uid=uid, aruco_id=int(mid), x=x, y=y, yaw=yaw))

            except Exception:
                pass

        if not blocks:
            self.get_logger().info("⏳ No blocks in TF (base_link).")
            return

        plan = compute_best_pickup_plan(
            blocks=blocks,
            cups_spacing_m=self.cups_spacing_m,
            tol_perp_m=self.tol_perp_m,
            tol_along_m=self.tol_along_m,
            tol_angle_deg=self.tol_angle_deg,
        )

        det_str = ", ".join([
            f"aruco{b.aruco_id}[{b.uid%1000}](x={b.x:.2f},y={b.y:.2f},yaw={math.degrees(b.yaw):.0f}°)"
            for b in blocks
        ])

        if plan is None:
            self.get_logger().info(f"👀 Blocks: {det_str} | ❌ No valid pickup plan (tolerances/orientation).")
            return

        theta_deg = math.degrees(plan.theta_arm)

        self.get_logger().info(
            f"👀 Blocks: {det_str}\n"
            f"✅ Best plan: score={plan.score}/4, theta_arm={theta_deg:.1f}°, u0={plan.u0:.3f} m, mean|v|={plan.mean_abs_v:.3f}\n"
            f"   matches (cup_index->uid): {plan.matches}"
        )


def main():
    rclpy.init()
    node = LocalPickupSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()