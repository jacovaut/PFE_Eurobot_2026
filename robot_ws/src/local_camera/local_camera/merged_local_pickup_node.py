#!/usr/bin/env python3
import json
import math
import socket
from dataclasses import dataclass
from itertools import combinations, permutations

import numpy as np
import rclpy
import rclpy.duration
import tf_transformations

from geometry_msgs.msg import Pose2D, TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


# =========================
# DATA STRUCTURES
# =========================
@dataclass
class XY:
    x: float
    y: float


@dataclass
class Block:
    name: str
    x: float
    y: float
    color: str = "unknown"
    yaw_deg: float = 0.0
    last_seen: float = 0.0
    raw_id: int = -1


@dataclass
class PickupCandidate:
    score: float
    picked_count: int
    avg_error: float
    yaw: float
    dx: float
    dy: float
    assignments: list


# =========================
# MATH
# =========================
def rotate_xy(p, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return XY(
        c * p.x - s * p.y,
        s * p.x + c * p.y
    )


def angle_between(a, b):
    return math.atan2(
        b.y - a.y,
        b.x - a.x
    )


def wrap_angle(a):
    return math.atan2(
        math.sin(a),
        math.cos(a)
    )


def rectangular_yaw_diff_deg(a, b):
    return abs(
        (a - b + 90.0) % 180.0 - 90.0
    )


# =========================
# SHARED SOLVE
# =========================
def solve_pose(
    cup_subset,
    block_subset
):
    n = len(cup_subset)

    if n == 1:
        yaw = 0.0
    else:
        (_, c1), (_, c2) = (
            cup_subset[0],
            cup_subset[-1]
        )

        (_, b1), (_, b2) = (
            block_subset[0],
            block_subset[-1]
        )

        yaw = wrap_angle(
            angle_between(b1, b2)
            - angle_between(c1, c2)
        )

    dx = 0.0
    dy = 0.0

    for (_, cxy), (_, b) in zip(
        cup_subset,
        block_subset
    ):
        rc = rotate_xy(cxy, yaw)

        dx += b.x - rc.x
        dy += b.y - rc.y

    dx /= n
    dy /= n

    return dx, dy, yaw


# =========================
# SOLVER
# =========================
def compute_best_pickup(
    cups,
    blocks,
    team_color="blue"
):
    MAX_ERROR = 0.040
    MAX_YAW = math.radians(105)
    MAX_BLOCK_YAW_DIFF_DEG = 20.0

    W_BLOCKS = 3000.0
    W_ERROR = 4000.0
    W_YAW = 10.0
    W_COLOR = 100.0
    W_BLOCK_PARALLEL = 25.0

    cup_items = list(cups.items())
    block_items = list(blocks.items())

    best = None

    for n in range(
        1,
        min(len(cup_items), len(block_items)) + 1
    ):
        for cup_subset in combinations(
            cup_items,
            n
        ):
            for block_subset in combinations(
                block_items,
                n
            ):

                yaw_spread = 0.0

                if n > 1:
                    ref = block_subset[0][1].yaw_deg

                    for _, b in block_subset[1:]:
                        diff = rectangular_yaw_diff_deg(
                            ref,
                            b.yaw_deg
                        )

                        yaw_spread = max(
                            yaw_spread,
                            diff
                        )

                        if diff > MAX_BLOCK_YAW_DIFF_DEG:
                            break
                    else:
                        pass
                    if diff > MAX_BLOCK_YAW_DIFF_DEG:
                        continue

                for perm in permutations(
                    block_subset
                ):
                    dx, dy, yaw = solve_pose(
                        cup_subset,
                        perm
                    )

                    if abs(yaw) > MAX_YAW:
                        continue

                    total_err = 0.0
                    color_score = 0.0
                    assigns = []
                    valid = True

                    for (cn, cxy), (_, b) in zip(
                        cup_subset,
                        perm
                    ):
                        rc = rotate_xy(cxy, yaw)

                        px = rc.x + dx
                        py = rc.y + dy

                        err = math.hypot(
                            px - b.x,
                            py - b.y
                        )

                        if err > MAX_ERROR:
                            valid = False
                            break

                        if b.color == team_color:
                            color_score += 1
                        elif b.color != "unknown":
                            color_score -= 0.5

                        total_err += err

                        assigns.append({
                            "cup": cn,
                            "block": b.name,
                            "color": b.color,
                            "err_m": err,
                            "err_mm": err * 1000.0,
                            "raw_id": b.raw_id,
                            "block_yaw_deg": b.yaw_deg
                        })

                    if not valid:
                        continue

                    avg_err = total_err / n

                    score = (
                        W_BLOCKS * n
                        - W_ERROR * avg_err
                        - W_YAW * abs(math.degrees(yaw))
                        + W_COLOR * color_score
                        - W_BLOCK_PARALLEL * yaw_spread
                    )

                    cand = PickupCandidate(
                        score=score,
                        picked_count=n,
                        avg_error=avg_err,
                        yaw=yaw,
                        dx=dx,
                        dy=dy,
                        assignments=assigns
                    )

                    if (
                        best is None
                        or cand.score > best.score
                    ):
                        best = cand

    return best


# =========================
# RECOMPUTE LOCKED POSE
# =========================
def recompute_locked_pose(
    cups,
    blocks,
    locked_assignments
):
    if not locked_assignments:
        return None

    cup_subset = []
    block_subset = []

    for a in locked_assignments:
        cup_name = a["cup"]
        block_name = a["block"]

        if (
            cup_name not in cups
            or block_name not in blocks
        ):
            return None

        cup_subset.append(
            (cup_name, cups[cup_name])
        )

        block_subset.append(
            (block_name, blocks[block_name])
        )

    dx, dy, yaw = solve_pose(
        cup_subset,
        block_subset
    )

    total_err = 0.0
    new_assignments = []

    for (cn, cxy), (_, b) in zip(
        cup_subset,
        block_subset
    ):
        rc = rotate_xy(cxy, yaw)

        px = rc.x + dx
        py = rc.y + dy

        err = math.hypot(
            px - b.x,
            py - b.y
        )

        total_err += err

        new_assignments.append({
            "cup": cn,
            "block": b.name,
            "color": b.color,
            "err_m": err,
            "err_mm": err * 1000.0,
            "raw_id": b.raw_id,
            "block_yaw_deg": b.yaw_deg
        })

    avg_err = total_err / len(
        locked_assignments
    )

    return PickupCandidate(
        score=0.0,
        picked_count=len(
            locked_assignments
        ),
        avg_error=avg_err,
        yaw=yaw,
        dx=dx,
        dy=dy,
        assignments=new_assignments
    )


# =========================
# NODE
# =========================
class MergedLocalPickupNode(
    rclpy.node.Node
):

    def __init__(self):
        super().__init__(
            "merged_local_pickup_node"
        )

        self.declare_parameter(
            "udp_port",
            5005
        )
        self.declare_parameter(
            "team_color",
            "blue"
        )
        self.declare_parameter(
            "block_timeout_sec",
            1.0
        )
        self.declare_parameter(
            "match_distance_m",
            0.035
        )

        self.udp_port = int(
            self.get_parameter(
                "udp_port"
            ).value
        )

        self.team_color = str(
            self.get_parameter(
                "team_color"
            ).value
        )

        self.block_timeout_sec = float(
            self.get_parameter(
                "block_timeout_sec"
            ).value
        )

        self.match_distance_m = float(
            self.get_parameter(
                "match_distance_m"
            ).value
        )

        self.id_to_color = {
            36: "blue",
            47: "yellow"
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )
        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        self.cup_frames = [
            "cup_0",
            "cup_1",
            "cup_2",
            "cup_3"
        ]

        self.tracked_blocks = {}

        self.next_index_by_color = {
            "blue": 0,
            "yellow": 0,
            "unknown": 0
        }

        self.lock_required_frames = 5

        self.reset_lock()

        self.locked_targets = {}
        self.current_cups = {}

        self.pickup_pose_pub = self.create_publisher(
            Pose2D,
            "/pickup_pose",
            10
        )

        self.best_pickup_pub = self.create_publisher(
            String,
            "/best_pickup",
            10
        )

        self.manip_info_pub = self.create_publisher(
            String,
            "/manip_info",
            10
        )

        self.pickup_status_sub = self.create_subscription(
            String,
            "/pickup_status",
            self.pickup_status_callback,
            10
        )

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.bind(
            ("0.0.0.0", self.udp_port)
        )

        self.sock.setblocking(False)

        self.timer = self.create_timer(
            0.05,
            self.loop
        )

        self.get_logger().info(
            "[SOLVER READY]"
        )

    # =========================
    # LOCK RESET
    # =========================
    def reset_lock(self):
        self.solution_locked = False
        self.locked_signature = None
        self.locked_best = None
        self.locked_targets = {}
        self.candidate_signature = None
        self.candidate_count = 0
        self.block_targets = {
            "cup_0": None,
            "cup_1": None,
            "cup_2": None,
            "cup_3": None,
        }

    # =========================
    # STATUS CALLBACK
    # =========================
    def pickup_status_callback(
        self,
        msg
    ):
        status = msg.data.lower().strip()

        if status in [
            "done",
            "unlock",
            "success",
            "failed",
            "aligned"
        ]:
            self.reset_lock()

    # =========================
    # SIGNATURE
    # =========================
    def get_signature(
        self,
        best
    ):
        return "|".join(
            sorted(
                f"{a['cup']}->{a['block']}"
                for a in best.assignments
            )
        )

    # =========================
    # TRACKER
    # =========================
    def make_name(
        self,
        color
    ):
        idx = self.next_index_by_color[
            color
        ]
        self.next_index_by_color[
            color
        ] += 1

        return f"{color}_{idx}"

    def update_tracking(
        self,
        detections
    ):
        now = (
            self.get_clock().now().nanoseconds
            * 1e-9
        )

        updated = set()

        for d in detections:
            best_name = None
            best_dist = None

            same_color = [
                (n, t)
                for n, t in self.tracked_blocks.items()
                if t.color == d.color
                and n not in updated
            ]

            for name, t in same_color:
                dist = math.hypot(
                    d.x - t.x,
                    d.y - t.y
                )

                if (
                    best_dist is None
                    or dist < best_dist
                ):
                    best_dist = dist
                    best_name = name

            if (
                best_name is not None
                and best_dist < self.match_distance_m
            ):
                t = self.tracked_blocks[
                    best_name
                ]

                t.x = d.x
                t.y = d.y
                t.yaw_deg = d.yaw_deg
                t.raw_id = d.raw_id
                t.last_seen = now

                updated.add(best_name)

            else:
                name = self.make_name(
                    d.color
                )

                self.tracked_blocks[
                    name
                ] = Block(
                    name=name,
                    x=d.x,
                    y=d.y,
                    color=d.color,
                    yaw_deg=d.yaw_deg,
                    last_seen=now,
                    raw_id=d.raw_id
                )

                updated.add(name)

        self.cleanup_stale_blocks(
            now
        )

        return self.tracked_blocks

    def cleanup_stale_blocks(
        self,
        now
    ):
        locked_names = set()

        if (
            self.solution_locked
            and self.locked_best
        ):
            for a in self.locked_best.assignments:
                locked_names.add(
                    a["block"]
                )

        to_del = []

        for name, b in self.tracked_blocks.items():

            if name in locked_names:
                continue

            if (
                now - b.last_seen
                > self.block_timeout_sec
            ):
                to_del.append(name)

        for name in to_del:
            self.get_logger().info(
                f"[TRACKER] Removing stale block: {name}"
            )
            del self.tracked_blocks[name]

    # =========================
    # TF PUBLISH
    # =========================
    def publish_block_transforms(
        self
    ):
        now_msg = (
            self.get_clock().now().to_msg()
        )

        for cup_name, b in self.block_targets.items():
            if b is None:
                continue

            slot_idx = cup_name.split("_")[1]
            frame_name = f"block_target_{slot_idx}"

            t = TransformStamped()

            t.header.stamp = now_msg
            t.header.frame_id = "base_link"
            t.child_frame_id = frame_name

            t.transform.translation.x = float(
                b.x
            )
            t.transform.translation.y = float(
                b.y
            )
            t.transform.translation.z = 0.0

            q = tf_transformations.quaternion_from_euler(
                0.0,
                0.0,
                math.radians(
                    b.yaw_deg
                )
            )

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(
                t
            )

    # =========================
    # LOCK LOGIC
    # =========================
    def update_lock(
        self,
        best
    ):
        if self.solution_locked:
            frozen_blocks = {}

            for a in self.locked_best.assignments:
                block_name = a["block"]

                if (
                    block_name
                    in self.tracked_blocks
                ):
                    b = self.tracked_blocks[
                        block_name
                    ]

                    self.locked_targets[
                        block_name
                    ] = Block(
                        name=b.name,
                        x=b.x,
                        y=b.y,
                        color=b.color,
                        yaw_deg=b.yaw_deg,
                        last_seen=b.last_seen,
                        raw_id=b.raw_id
                    )

                if (
                    block_name
                    in self.locked_targets
                ):
                    frozen_blocks[
                        block_name
                    ] = self.locked_targets[
                        block_name
                    ]

            updated = recompute_locked_pose(
                self.current_cups,
                frozen_blocks,
                self.locked_best.assignments
            )

            if updated:
                self.locked_best.dx = updated.dx
                self.locked_best.dy = updated.dy
                self.locked_best.yaw = updated.yaw
                self.locked_best.avg_error = (
                    updated.avg_error
                )
                self.locked_best.assignments = (
                    updated.assignments
                )

            return self.locked_best

        if best is None:
            self.candidate_signature = None
            self.candidate_count = 0
            return None

        sig = self.get_signature(
            best
        )

        if sig == self.candidate_signature:
            self.candidate_count += 1
        else:
            self.candidate_signature = sig
            self.candidate_count = 1

        if (
            self.candidate_count
            >= self.lock_required_frames
        ):
            self.solution_locked = True
            self.locked_signature = sig
            self.locked_best = best
            self.locked_targets = {}

            for a in best.assignments:
                block_name = a["block"]

                if (
                    block_name
                    in self.tracked_blocks
                ):
                    b = self.tracked_blocks[
                        block_name
                    ]

                    self.locked_targets[
                        block_name
                    ] = Block(
                        name=b.name,
                        x=b.x,
                        y=b.y,
                        color=b.color,
                        yaw_deg=b.yaw_deg,
                        last_seen=b.last_seen,
                        raw_id=b.raw_id
                    )

            self.get_logger().info(
                f"[LOCKED] {sig}"
            )

            return best

        self.get_logger().info(
            f"[CANDIDATE] {sig} "
            f"({self.candidate_count}/"
            f"{self.lock_required_frames})"
        )

        return None

    # =========================
    # MAIN LOOP
    # =========================
    def loop(self):
        cups = {}

        for name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",
                    name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(
                        seconds=0.005
                    )
                )

                t = tf.transform.translation

                cups[name] = XY(
                    t.x,
                    t.y
                )

            except Exception:
                return

        self.current_cups = cups

        latest = None

        while True:
            try:
                data, _ = self.sock.recvfrom(
                    4096
                )
                latest = data
            except BlockingIOError:
                break

        raw = []

        if latest:
            raw = json.loads(
                latest.decode()
            )

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                "arducam_optical_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=0.005
                )
            )
        except Exception:
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        T = tf_transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w]
        )

        T[0:3, 3] = [
            t.x,
            t.y,
            t.z
        ]

        detections = []

        for b in raw:
            try:
                raw_id = int(
                    b.get("id", -1)
                )

                color = self.id_to_color.get(
                    raw_id,
                    "unknown"
                )

                yaw = float(
                    b.get(
                        "yaw_deg",
                        0.0
                    )
                )

                p = np.array([
                    float(b["x_cam"]),
                    float(b["y_cam"]),
                    float(b["z_cam"]),
                    1.0
                ])

                pb = T @ p

                detections.append(
                    Block(
                        name="d",
                        x=pb[0],
                        y=pb[1],
                        color=color,
                        yaw_deg=yaw,
                        raw_id=raw_id
                    )
                )

            except Exception:
                continue

        blocks = self.update_tracking(
            detections
        )

        self.publish_block_transforms()

        if not blocks:
            return

        best = compute_best_pickup(
            cups,
            blocks,
            self.team_color
        )

        locked = self.update_lock(
            best
        )

        if not locked:
            return

        self.publish_best_pickup(
            locked
        )

    # =========================
    # PUBLISH
    # =========================
    def publish_best_pickup(
        self,
        best
    ):
        for key in self.block_targets:
            self.block_targets[key] = None

        for a in best.assignments:
            cup_name = a["cup"]
            block_name = a["block"]
            if block_name in self.locked_targets:
                self.block_targets[
                    cup_name
                ] = self.locked_targets[block_name]
            elif block_name in self.tracked_blocks:
                self.block_targets[
                    cup_name
                ] = self.tracked_blocks[block_name]

        pose = Pose2D()

        pose.x = best.dx
        pose.y = best.dy
        pose.theta = best.yaw

        self.pickup_pose_pub.publish(
            pose
        )

        msg = String()

        msg.data = json.dumps({
            "dx": best.dx,
            "dy": best.dy,
            "yaw_deg": math.degrees(
                best.yaw
            ),
            "assignments": best.assignments
        })

        self.best_pickup_pub.publish(
            msg
        )

        pump = {
            "cup_0": "",
            "cup_1": "",
            "cup_2": "",
            "cup_3": ""
        }

        color_map = {
            "blue": "B",
            "yellow": "Y"
        }

        for a in best.assignments:
            pump[a["cup"]] = color_map.get(
                a["color"],
                ""
            )

        manip = String()

        manip.data = (
            f"P1={pump['cup_0']} ,"
            f"P2={pump['cup_1']} ,"
            f"P3={pump['cup_2']} ,"
            f"P4={pump['cup_3']}"
        )

        self.manip_info_pub.publish(
            manip
        )


# =========================
# MAIN
# =========================
def main():
    rclpy.init()

    node = MergedLocalPickupNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()