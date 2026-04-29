#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.duration

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import socket
import json
import math
import numpy as np
import tf_transformations

from dataclasses import dataclass
from itertools import combinations, permutations


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
    c = math.cos(yaw)
    s = math.sin(yaw)
    return XY(c*p.x - s*p.y, s*p.x + c*p.y)


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def wrap_angle(a):
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a


def rectangular_yaw_diff_deg(a, b):
    return abs((a - b + 90.0) % 180.0 - 90.0)


# =========================
# SOLVER
# =========================
def compute_best_pickup(cups, blocks, team_color="blue"):

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

    for n in range(1, min(len(cup_items), len(block_items)) + 1):

        for cup_subset in combinations(cup_items, n):
            for block_subset in combinations(block_items, n):

                # --- YAW CONSISTENCY ---
                if n > 1:
                    ref = block_subset[0][1].yaw_deg
                    yaw_spread = 0.0
                    valid_parallel = True

                    for _, b in block_subset[1:]:
                        diff = rectangular_yaw_diff_deg(ref, b.yaw_deg)
                        yaw_spread = max(yaw_spread, diff)

                        if diff > MAX_BLOCK_YAW_DIFF_DEG:
                            valid_parallel = False
                            break

                    if not valid_parallel:
                        continue
                else:
                    yaw_spread = 0.0

                for perm in permutations(block_subset):

                    if n == 1:
                        yaw = 0.0
                    else:
                        (_, c1), (_, c2) = cup_subset[0], cup_subset[-1]
                        (_, b1), (_, b2) = perm[0], perm[-1]

                        yaw = wrap_angle(
                            angle_between(b1, b2) - angle_between(c1, c2)
                        )

                    if abs(yaw) > MAX_YAW:
                        continue

                    dx = dy = 0.0

                    for (cn, cxy), (bn, b) in zip(cup_subset, perm):
                        rc = rotate_xy(cxy, yaw)
                        dx += b.x - rc.x
                        dy += b.y - rc.y

                    dx /= n
                    dy /= n

                    total_err = 0.0
                    color_score = 0.0
                    valid = True
                    assigns = []

                    for (cn, cxy), (bn, b) in zip(cup_subset, perm):
                        rc = rotate_xy(cxy, yaw)

                        px = rc.x + dx
                        py = rc.y + dy

                        err = math.hypot(px - b.x, py - b.y)

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
                            "err_mm": err * 1000,
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
                        score, n, avg_err, yaw, dx, dy, assigns
                    )

                    if best is None or cand.score > best.score:
                        best = cand

    return best


# =========================
# NODE
# =========================
class MergedLocalPickupNode(Node):

    def __init__(self):
        super().__init__('merged_local_pickup_node')

        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('team_color', 'blue')
        self.declare_parameter('block_timeout_sec', 1.0)
        self.declare_parameter('match_distance_m', 0.035)

        self.udp_port = int(self.get_parameter('udp_port').value)
        self.team_color = str(self.get_parameter('team_color').value)
        self.block_timeout_sec = float(self.get_parameter('block_timeout_sec').value)
        self.match_distance_m = float(self.get_parameter('match_distance_m').value)

        # ID → color
        self.id_to_color = {36: "blue", 47: "yellow"}

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]

        # tracking
        self.tracked_blocks = {}
        self.next_index_by_color = {"blue": 0, "yellow": 0, "unknown": 0}

        # publishers
        self.pickup_pose_pub = self.create_publisher(Pose2D, "/pickup_pose", 10)
        self.best_pickup_pub = self.create_publisher(String, "/best_pickup", 10)
        self.manip_info_pub = self.create_publisher(String, "/manip_info", 10)

        # subscriber for unlock
        self.pickup_status_sub = self.create_subscription(
            String,
            "/pickup_status",
            self.pickup_status_callback,
            10
        )

        # confidence lock
        self.lock_required_frames = 5
        self.candidate_signature = None
        self.candidate_count = 0
        self.locked_signature = None
        self.locked_best = None
        self.solution_locked = False

        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.udp_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("[SOLVER READY]")

    # =========================
    # LOCK CALLBACK
    # =========================
    def pickup_status_callback(self, msg):
        status = msg.data.lower().strip()

        if status in ["done", "unlock", "success", "failed"]:
            self.get_logger().info(f"[LOCK] released by robot: {status}")

            self.solution_locked = False
            self.locked_signature = None
            self.locked_best = None
            self.candidate_signature = None
            self.candidate_count = 0

    # =========================
    # LOCK LOGIC
    # =========================
    def get_signature(self, best):
        return "|".join(
            sorted([f"{a['cup']}->{a['block']}" for a in best.assignments])
        )

    def update_lock(self, best):

        if self.solution_locked:
            return self.locked_best

        if best is None:
            self.candidate_signature = None
            self.candidate_count = 0
            return None

        sig = self.get_signature(best)

        if sig == self.candidate_signature:
            self.candidate_count += 1
        else:
            self.candidate_signature = sig
            self.candidate_count = 1

        if self.candidate_count >= self.lock_required_frames:
            self.solution_locked = True
            self.locked_signature = sig
            self.locked_best = best

            self.get_logger().info(f"[LOCKED] {sig}")
            return best

        self.get_logger().info(f"[CANDIDATE] {sig} ({self.candidate_count}/5)")
        return None

    # =========================
    # TRACKING
    # =========================
    def make_name(self, color):
        idx = self.next_index_by_color[color]
        self.next_index_by_color[color] += 1
        return f"{color}_{idx}"

    def update_tracking(self, detections):

        now = self.get_clock().now().nanoseconds * 1e-9

        updated = set()

        for d in detections:
            best_name = None
            best_dist = None

            for name, t in self.tracked_blocks.items():
                if t.color != d.color or name in updated:
                    continue

                dist = math.hypot(d.x - t.x, d.y - t.y)

                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_name = name

            if best_name is not None and best_dist < self.match_distance_m:
                t = self.tracked_blocks[best_name]
                t.x = d.x
                t.y = d.y
                t.yaw_deg = d.yaw_deg
                t.raw_id = d.raw_id
                t.last_seen = now
                updated.add(best_name)

            else:
                name = self.make_name(d.color)

                self.tracked_blocks[name] = Block(
                    name=name,
                    x=d.x,
                    y=d.y,
                    color=d.color,
                    yaw_deg=d.yaw_deg,
                    last_seen=now,
                    raw_id=d.raw_id
                )

                updated.add(name)

        to_del = []

        for name, b in self.tracked_blocks.items():
            if now - b.last_seen > self.block_timeout_sec:
                to_del.append(name)

        for name in to_del:
            del self.tracked_blocks[name]

        return self.tracked_blocks
    
   
    # =========================
    # LOOP
    # =========================
    def loop(self):

        # cups
        cups = {}
        for name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link", name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.005)
                )
                t = tf.transform.translation
                cups[name] = XY(t.x, t.y)
            except:
                return

        # UDP
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                latest = data
            except BlockingIOError:
                break

        raw = []
        if latest:
            raw = json.loads(latest.decode())

        # TF cam
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", "arducam_optical_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.005)
            )
        except:
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [t.x, t.y, t.z]

        detections = []

        for b in raw:
            try:
                raw_id = int(b.get("id", -1))
                color = self.id_to_color.get(raw_id, "unknown")
                yaw = float(b.get("yaw_deg", 0.0))

                p = np.array([
                    float(b["x_cam"]),
                    float(b["y_cam"]),
                    float(b["z_cam"]),
                    1.0
                ])

                pb = T @ p

                detections.append(Block(
                    name="d",
                    x=pb[0],
                    y=pb[1],
                    color=color,
                    yaw_deg=yaw,
                    last_seen=0.0,
                    raw_id=raw_id
                ))

            except:
                continue

        blocks = self.update_tracking(detections)

        if not blocks:
            return

        best = compute_best_pickup(cups, blocks, self.team_color)

        locked = self.update_lock(best)

        if not locked:
            return

        self.publish_best_pickup(locked)

    # =========================
    # PUBLISH
    # =========================
    def publish_best_pickup(self, best):

        pose = Pose2D()
        pose.x = best.dx
        pose.y = best.dy
        pose.theta = best.yaw
        self.pickup_pose_pub.publish(pose)

        msg = String()
        msg.data = json.dumps({
            "dx": best.dx,
            "dy": best.dy,
            "yaw_deg": math.degrees(best.yaw),
            "assignments": best.assignments
        })
        self.best_pickup_pub.publish(msg)

        pump = {"cup_0": "", "cup_1": "", "cup_2": "", "cup_3": ""}

        for a in best.assignments:
            if a["color"] == "blue":
                pump[a["cup"]] = "B"
            elif a["color"] == "yellow":
                pump[a["cup"]] = "Y"

        s = f"P1={pump['cup_0']} ,P2={pump['cup_1']} ,P3={pump['cup_2']} ,P4={pump['cup_3']}"

        m = String()
        m.data = s
        self.manip_info_pub.publish(m)


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