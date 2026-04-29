#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.duration
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
# SOLVER MATH
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


def compute_best_pickup(cups, blocks):
    MAX_ERROR = 2.0
    MAX_YAW = math.radians(90)

    W_BLOCKS = 100.0
    W_ERROR = 80.0
    W_YAW = 5.0

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

                    dx = dy = 0.0

                    for (cn, cxy), (bn, b) in zip(cup_subset, perm):
                        rc = rotate_xy(cxy, yaw)
                        dx += b.x - rc.x
                        dy += b.y - rc.y

                    dx /= n
                    dy /= n

                    total_err = 0.0
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

                        total_err += err
                        assigns.append((cn, bn, err))

                    if not valid:
                        continue

                    avg_err = total_err / n

                    score = (
                        W_BLOCKS * n
                        - W_ERROR * avg_err
                        - W_YAW * abs(yaw)
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
        self.udp_port = int(self.get_parameter('udp_port').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]

        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.sock.bind(("0.0.0.0", self.udp_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("[SOLVER] READY")

    # =========================
    # PIXEL → BASE_LINK
    # =========================
    def pixel_to_base(self, cx, cy):

        fx = 457.33917579
        fy = 453.81772548
        cx0 = 637.592287
        cy0 = 374.90978642

        x = (cx - cx0) / fx
        y = (cy - cy0) / fy

        ray_cam = np.array([x, y, 1.0, 0.0])
        origin_cam = np.array([0, 0, 0, 1])

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                "arducam_optical_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.005)
            )
        except:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        quat = [q.x, q.y, q.z, q.w]
        T = tf_transformations.quaternion_matrix(quat)
        T[0:3, 3] = [t.x, t.y, t.z]

        origin = T @ origin_cam
        ray = T @ ray_cam

        d = ray[:3]

        if abs(d[2]) < 1e-6:
            return None

        TABLE_Z = -0.153

        s = (TABLE_Z - origin[2]) / d[2]

        # If projection is behind camera, flip ray direction
        if s < 0:
            d = -d
            s = (TABLE_Z - origin[2]) / d[2]

        if s < 0:
            return None

        p = origin[:3] + s * d

        return XY(float(p[0]), float(p[1]))

    # =========================
    # LOOP
    # =========================
    def loop(self):

        # ---- Cups ----
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

        # ---- UDP drain ----
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                latest = data
            except BlockingIOError:
                break

        if latest is None:
            return

        try:
            raw_blocks = json.loads(latest.decode())
        except:
            return

        # ---- Convert blocks ----
        blocks = {}

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

        for b in raw_blocks:
            if "x_cam" not in b:
                continue

            p_cam = np.array([
                float(b["x_cam"]),
                float(b["y_cam"]),
                float(b["z_cam"]),
                1.0
            ])

            p_base = T @ p_cam

            name = f"block_{b['id']}"

            blocks[name] = Block(
                name=name,
                x=float(p_base[0]),
                y=float(p_base[1])
            )

        self.get_logger().info(f"[SOLVER] PnP blocks: {blocks}")
        
        if not blocks:
            return

        # ---- Solve ----
        best = compute_best_pickup(cups, blocks)

        if best is None:
            return

        self.get_logger().info(
            f"BEST | pick={best.picked_count} "
            f"| score={best.score:.1f} "
            f"| err={best.avg_error*1000:.1f}mm "
            f"| yaw={math.degrees(best.yaw):.1f}deg"
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