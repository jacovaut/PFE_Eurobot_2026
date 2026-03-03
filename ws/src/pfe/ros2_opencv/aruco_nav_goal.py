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
    id: int
    x: float        # in base_link (m)
    y: float        # in base_link (m)
    yaw: float      # block yaw in base_link (rad)


@dataclass
class PickupPlan:
    theta_arm: float                 # desired arm axis yaw in base_link (rad)
    u0: float                        # center shift along arm axis (m)
    matches: Dict[int, int]          # cup_index -> block_id
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
    """
    Principal direction of XY points (radians). Good guess for row direction.
    """
    if len(points) < 2:
        return None
    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)

    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)

    # tan(2a) = 2sxy / (sxx - syy)
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
        # fallback: face closest block direction
        b0 = min(blocks, key=lambda b: b.x * b.x + b.y * b.y)
        base_theta = math.atan2(b0.y, b0.x)

    # Candidate arm yaws (keep list small for speed)
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

        # Rotate blocks into arm frame: u along arm, v perpendicular
        arm_blocks = []
        for b in blocks:
            u = c * b.x + s * b.y
            v = -s * b.x + c * b.y

            if abs(v) > tol_perp_m:
                continue
            if not is_perpendicular(b.yaw, theta, tol_angle):
                continue

            arm_blocks.append((b.id, u, v))

        if not arm_blocks:
            continue

        # Enumerate shift candidates u0 = u_block - cup_offset
        for (bid, u, _v) in arm_blocks:
            for k, cup_u in enumerate(cups):
                u0 = u - cup_u

                # Greedy match: each cup -> closest unused block within tol_along
                unused = {ab[0] for ab in arm_blocks}
                matches: Dict[int, int] = {}
                abs_v_list = []

                for cup_idx, cup_off in enumerate(cups):
                    target_u = u0 + cup_off

                    best_blk = None
                    best_du = None
                    best_v = None
                    for (bid2, u2, v2) in arm_blocks:
                        if bid2 not in unused:
                            continue
                        du = abs(u2 - target_u)
                        if du <= tol_along_m and (best_du is None or du < best_du):
                            best_du = du
                            best_blk = bid2
                            best_v = v2

                    if best_blk is not None:
                        matches[cup_idx] = best_blk
                        unused.remove(best_blk)
                        abs_v_list.append(abs(best_v))

                score = len(matches)
                if score == 0:
                    continue

                mean_abs_v = sum(abs_v_list) / len(abs_v_list) if abs_v_list else 1e9

                plan = PickupPlan(
                    theta_arm=theta,
                    u0=u0,
                    matches=matches,
                    score=score,
                    mean_abs_v=mean_abs_v
                )

                # Best = max score; tie-break = tighter lateral alignment
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

        # Which markers to consider (edit these for your setup)
        self.aruco_ids = self.declare_parameter('aruco_ids', [1, 2, 3, 4, 47, 48, 49, 50]).value

        # Solver params
        self.cups_spacing_m = float(self.declare_parameter('cups_spacing_m', 0.05).value)
        self.tol_perp_m = float(self.declare_parameter('tol_perp_m', 0.02).value)
        self.tol_along_m = float(self.declare_parameter('tol_along_m', 0.02).value)
        self.tol_angle_deg = float(self.declare_parameter('tol_angle_deg', 15.0).value)

        # Filter window (optional but helpful)
        self.max_dist_m = float(self.declare_parameter('max_dist_m', 1.5).value)
        self.min_x_m = float(self.declare_parameter('min_x_m', 0.05).value)  # ignore behind/too close

        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz printing

        self.get_logger().info("✅ LocalPickupSolver running. Watching TF for ArUco markers...")

    def tick(self):
        blocks: List[BlockObs] = []

        for mid in self.aruco_ids:
            target_frame = f"{self.aruco_prefix}{mid}"
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    target_frame,
                    rclpy.time.Time()
                )

                x = float(t.transform.translation.x)
                y = float(t.transform.translation.y)

                # basic range filtering
                if x < self.min_x_m:
                    continue
                if math.hypot(x, y) > self.max_dist_m:
                    continue

                q = t.transform.rotation
                # yaw from quaternion
                _roll, _pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                blocks.append(BlockObs(id=int(mid), x=x, y=y, yaw=yaw))

            except Exception:
                # marker not available right now
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

        # Print detections
        det_str = ", ".join([f"id{b.id}(x={b.x:.2f},y={b.y:.2f},yaw={math.degrees(b.yaw):.0f}°)" for b in blocks])

        if plan is None:
            self.get_logger().info(f"👀 Blocks: {det_str} | ❌ No valid pickup plan (tolerances/orientation).")
            return

        theta_deg = math.degrees(plan.theta_arm)
        # u0 is along the arm axis; because arm axis = robot +x when theta=0, u0 is a "forward distance" proxy
        self.get_logger().info(
            f"👀 Blocks: {det_str}\n"
            f"✅ Best plan: score={plan.score}/4, theta_arm={theta_deg:.1f}°, u0={plan.u0:.3f} m, mean|v|={plan.mean_abs_v:.3f}\n"
            f"   matches (cup_index->aruco_id): {plan.matches}"
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