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

from dataclasses import dataclass, field
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
# MATH HELPERS
# =========================
def rotate_xy(p, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return XY(c * p.x - s * p.y, s * p.x + c * p.y)


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# =========================
# SOLVER
# =========================
def compute_best_pickup(cups, blocks, team_color="blue"):
    MAX_ERROR = 0.03
    MAX_YAW = math.radians(90)

    W_BLOCKS = 1000.0
    W_ERROR = 8000.0
    W_YAW = 200.0
    W_COLOR = 1000.0

    cup_items = list(cups.items())
    block_items = list(blocks.items())

    best = None

    for n in range(1, min(len(cup_items), len(block_items)) + 1):
        for cup_subset in combinations(cup_items, n):
            for block_subset in combinations(block_items, n):
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

                    dx = 0.0
                    dy = 0.0

                    for (cup_name, cup_xy), (block_name, block) in zip(cup_subset, perm):
                        rc = rotate_xy(cup_xy, yaw)
                        dx += block.x - rc.x
                        dy += block.y - rc.y

                    dx /= n
                    dy /= n

                    total_err = 0.0
                    color_score = 0.0
                    valid = True
                    assignments = []

                    for (cup_name, cup_xy), (block_name, block) in zip(cup_subset, perm):
                        rc = rotate_xy(cup_xy, yaw)

                        projected_x = rc.x + dx
                        projected_y = rc.y + dy

                        err = math.hypot(projected_x - block.x, projected_y - block.y)

                        if err > MAX_ERROR:
                            valid = False
                            break

                        if block.color == team_color:
                            color_score += 1.0
                        elif block.color == "unknown":
                            color_score += 0.0
                        else:
                            color_score -= 0.5

                        total_err += err

                        assignments.append({
                            "cup": cup_name,
                            "block": block.name,
                            "color": block.color,
                            "err_m": err,
                            "raw_id": block.raw_id,
                        })

                    if not valid:
                        continue

                    avg_err = total_err / n

                    score = (
                        W_BLOCKS * n
                        - W_ERROR * avg_err
                        - W_YAW * abs(math.degrees(yaw))
                        + W_COLOR * color_score
                    )

                    candidate = PickupCandidate(
                        score=score,
                        picked_count=n,
                        avg_error=avg_err,
                        yaw=yaw,
                        dx=dx,
                        dy=dy,
                        assignments=assignments
                    )

                    if best is None or candidate.score > best.score:
                        best = candidate

    return best


# =========================
# NODE
# =========================
class MergedLocalPickupNode(Node):

    def __init__(self):
        super().__init__("merged_local_pickup_node")

        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("team_color", "blue")
        self.declare_parameter("block_timeout_sec", 1.0)
        self.declare_parameter("match_distance_m", 0.035)

        self.udp_port = int(self.get_parameter("udp_port").value)
        self.team_color = str(self.get_parameter("team_color").value)
        self.block_timeout_sec = float(self.get_parameter("block_timeout_sec").value)
        self.match_distance_m = float(self.get_parameter("match_distance_m").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]
        
        self.id_to_color = {
            36: "blue",
            47: "yellow",
        }

        self.tracked_blocks = {}
        self.next_index_by_color = {
            "blue": 0,
            "yellow": 0,
            "unknown": 0,
        }

        self.pickup_pose_pub = self.create_publisher(Pose2D, "/pickup_pose", 10)
        self.best_pickup_pub = self.create_publisher(String, "/best_pickup", 10)
        self.manip_info_pub = self.create_publisher(String, "/manip_info", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.sock.bind(("0.0.0.0", self.udp_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("[SOLVER] READY")
        self.get_logger().info(f"[SOLVER] team_color={self.team_color}")

    # =========================
    # TRACKING
    # =========================
    def normalize_color(self, color):
        color = str(color).lower().strip()

        if color in ["blue", "b", "bleu"]:
            return "blue"
        if color in ["yellow", "y", "jaune"]:
            return "yellow"

        return "unknown"

    def make_block_name(self, color):
        if color not in self.next_index_by_color:
            self.next_index_by_color[color] = 0

        idx = self.next_index_by_color[color]
        self.next_index_by_color[color] += 1

        return f"{color}_{idx}"

    def update_tracked_blocks(self, detections):
        """
        detections = list of temporary Block objects from current camera frame.

        This keeps block names stable.
        If a block disappears, it stays alive for block_timeout_sec.
        """

        now = self.get_clock().now().nanoseconds * 1e-9
        updated_names = set()

        for det in detections:
            best_name = None
            best_dist = None

            for name, tracked in self.tracked_blocks.items():
                if tracked.color != det.color:
                    continue

                if name in updated_names:
                    continue

                dist = math.hypot(det.x - tracked.x, det.y - tracked.y)

                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_name = name

            if best_name is not None and best_dist <= self.match_distance_m:
                tracked = self.tracked_blocks[best_name]
                tracked.x = det.x
                tracked.y = det.y
                tracked.last_seen = now
                tracked.raw_id = det.raw_id
                updated_names.add(best_name)

            else:
                new_name = self.make_block_name(det.color)

                self.tracked_blocks[new_name] = Block(
                    name=new_name,
                    x=det.x,
                    y=det.y,
                    color=det.color,
                    last_seen=now,
                    raw_id=det.raw_id
                )

                updated_names.add(new_name)

        # Delete blocks only after timeout
        names_to_delete = []

        for name, block in self.tracked_blocks.items():
            age = now - block.last_seen

            if age > self.block_timeout_sec:
                names_to_delete.append(name)

        for name in names_to_delete:
            del self.tracked_blocks[name]

        return self.tracked_blocks

    # =========================
    # LOOP
    # =========================
    def loop(self):

        # ---- Get cup positions from TF ----
        cups = {}

        for name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",
                    name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.005)
                )

                t = tf.transform.translation
                cups[name] = XY(float(t.x), float(t.y))

            except Exception:
                return

        # ---- Drain UDP and keep latest packet ----
        latest = None

        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                latest = data
            except BlockingIOError:
                break

        # Important:
        # If no latest packet, do NOT delete tracked blocks immediately.
        # Just let timeout logic handle it.
        raw_blocks = []

        if latest is not None:
            try:
                raw_blocks = json.loads(latest.decode())

                if isinstance(raw_blocks, dict):
                    raw_blocks = [raw_blocks]

            except Exception as e:
                self.get_logger().warn(f"Bad UDP JSON: {e}")
                raw_blocks = []

        # ---- Camera TF ----
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                "arducam_optical_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.005)
            )

        except Exception as e:
            self.get_logger().warn(f"Missing camera TF: {e}")
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        quat = [q.x, q.y, q.z, q.w]
        T = tf_transformations.quaternion_matrix(quat)
        T[0:3, 3] = [t.x, t.y, t.z]

        # ---- Convert detected blocks to base_link ----
        detections = []

        for b in raw_blocks:
            if "x_cam" not in b or "y_cam" not in b or "z_cam" not in b:
                continue

            try:
                raw_id = int(b.get("id", -1))
                
                raw_id = int(b.get("id", -1))

                if "color" in b:
                    color = self.normalize_color(b.get("color", "unknown"))
                else:
                    color = self.id_to_color.get(raw_id, "unknown")

                p_cam = np.array([
                    float(b["x_cam"]),
                    float(b["y_cam"]),
                    float(b["z_cam"]),
                    1.0
                ])

                p_base = T @ p_cam

                detections.append(Block(
                    name="detection",
                    x=float(p_base[0]),
                    y=float(p_base[1]),
                    color=color,
                    raw_id=raw_id
                ))

            except Exception as e:
                self.get_logger().warn(f"Bad block data skipped: {e}")
                continue

        # ---- Update stable tracked blocks ----
        blocks = self.update_tracked_blocks(detections)

        if not blocks:
            return

        self.get_logger().info(f"[SOLVER] tracked blocks: {blocks}")

        # ---- Solve ----
        best = compute_best_pickup(
            cups=cups,
            blocks=blocks,
            team_color=self.team_color
        )

        if best is None:
            return

        # ---- Publish actionable command ----
        self.publish_best_pickup(best)

        self.get_logger().info(
            f"BEST | pick={best.picked_count} "
            f"| score={best.score:.1f} "
            f"| err={best.avg_error * 1000:.1f}mm "
            f"| yaw={math.degrees(best.yaw):.1f}deg "
            f"| dx={best.dx:.3f} "
            f"| dy={best.dy:.3f}"
        )

    # =========================
    # PUBLISHERS
    # =========================
    def publish_best_pickup(self, best):

        pose = Pose2D()
        pose.x = float(best.dx)
        pose.y = float(best.dy)
        pose.theta = float(best.yaw)

        self.pickup_pose_pub.publish(pose)

        msg_data = {
            "pickup_pose": {
                "x": best.dx,
                "y": best.dy,
                "yaw_rad": best.yaw,
                "yaw_deg": math.degrees(best.yaw),
            },
            "score": best.score,
            "picked_count": best.picked_count,
            "avg_error_m": best.avg_error,
            "avg_error_mm": best.avg_error * 1000.0,
            "team_color": self.team_color,
            "assignments": best.assignments
        }

        msg = String()
        msg.data = json.dumps(msg_data)

        self.best_pickup_pub.publish(msg)

        pump_colors = {
            "cup_0": "",
            "cup_1": "",
            "cup_2": "",
            "cup_3": "",
        }

        for a in best.assignments:
            color = a.get("color", "unknown")

            if color == "blue":
                pump_colors[a["cup"]] = "B"
            elif color == "yellow":
                pump_colors[a["cup"]] = "Y"
            else:
                pump_colors[a["cup"]] = "U"

        manip_text = (
            f"P1={pump_colors['cup_0']} ,"
            f"P2={pump_colors['cup_1']} ,"
            f"P3={pump_colors['cup_2']} ,"
            f"P4={pump_colors['cup_3']}"
        )

        manip_msg = String()
        manip_msg.data = manip_text
        self.manip_info_pub.publish(manip_msg)

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