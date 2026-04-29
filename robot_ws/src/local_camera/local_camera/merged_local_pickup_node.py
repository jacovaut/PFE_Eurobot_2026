#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.duration
from tf2_ros import Buffer, TransformListener
import socket
import json
import math
from dataclasses import dataclass
from itertools import combinations, permutations


# ==========================================================
# DATA CLASSES
# ==========================================================
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
    assignments: list  # [(cup_name, block_name, error), ...]


# ==========================================================
# SOLVER HELPERS
# ==========================================================
def rotate_xy(p: XY, yaw: float) -> XY:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return XY(
        c * p.x - s * p.y,
        s * p.x + c * p.y
    )


def angle_between(a: XY, b: XY) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def compute_best_pickup(cups: dict, blocks: dict):
    """
    cups: dict[str, XY]
    blocks: dict[str, Block]

    NOTE:
    This assumes cups and blocks are in the SAME coordinate system.
    Ideally: meters in base_link.
    """

    MAX_ERROR = 0.025          # 25 mm
    MAX_ARM_YAW = math.radians(90)

    W_BLOCK_COUNT = 100.0
    W_ALIGNMENT_ERROR = 80.0
    W_YAW = 5.0

    cup_items = list(cups.items())
    block_items = list(blocks.items())

    best = None
    max_pick = min(len(cup_items), len(block_items))

    for n in range(1, max_pick + 1):
        for cup_subset in combinations(cup_items, n):
            for block_subset in combinations(block_items, n):
                for block_perm in permutations(block_subset):

                    assignments_raw = list(zip(cup_subset, block_perm))

                    if n == 1:
                        yaw = 0.0
                    else:
                        (_, cup_a), (_, cup_b) = cup_subset[0], cup_subset[-1]
                        (_, block_a), (_, block_b) = block_perm[0], block_perm[-1]

                        cup_angle = angle_between(cup_a, cup_b)
                        block_angle = angle_between(
                            XY(block_a.x, block_a.y),
                            XY(block_b.x, block_b.y)
                        )

                        yaw = wrap_angle(block_angle - cup_angle)

                    if abs(yaw) > MAX_ARM_YAW:
                        continue

                    dx_sum = 0.0
                    dy_sum = 0.0

                    for (cup_name, cup_xy), (block_name, block) in assignments_raw:
                        rc = rotate_xy(cup_xy, yaw)
                        dx_sum += block.x - rc.x
                        dy_sum += block.y - rc.y

                    dx = dx_sum / n
                    dy = dy_sum / n

                    total_error = 0.0
                    assignments = []
                    valid = True

                    for (cup_name, cup_xy), (block_name, block) in assignments_raw:
                        rc = rotate_xy(cup_xy, yaw)

                        predicted_x = rc.x + dx
                        predicted_y = rc.y + dy

                        error = math.hypot(
                            predicted_x - block.x,
                            predicted_y - block.y
                        )

                        if error > MAX_ERROR:
                            valid = False
                            break

                        total_error += error
                        assignments.append((cup_name, block.name, error))

                    if not valid:
                        continue

                    avg_error = total_error / n

                    score = (
                        W_BLOCK_COUNT * n
                        - W_ALIGNMENT_ERROR * avg_error
                        - W_YAW * abs(yaw)
                    )

                    candidate = PickupCandidate(
                        score=score,
                        picked_count=n,
                        avg_error=avg_error,
                        yaw=yaw,
                        dx=dx,
                        dy=dy,
                        assignments=assignments
                    )

                    if best is None or candidate.score > best.score:
                        best = candidate

    return best


# ==========================================================
# NODE
# ==========================================================
class MergedLocalPickupNode(Node):
    def __init__(self):
        super().__init__('merged_local_pickup_node')

        # Parameters
        self.declare_parameter('udp_port', 5005)
        self.udp_port = int(self.get_parameter('udp_port').value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cups from URDF / robot_state_publisher
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]

        # UDP socket
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.udp_sock.bind(("0.0.0.0", self.udp_port))
        self.udp_sock.setblocking(False)

        # 20 Hz loop
        self.timer = self.create_timer(0.05, self.state_machine)

        self.get_logger().info("[MERGED] Local pickup solver started.")

    # ------------------------------------------------------
    # Get cups in base_link
    # ------------------------------------------------------
    def get_cups(self):
        cups = {}

        for cup_name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",
                    cup_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.005)
                )

                t = tf.transform.translation
                cups[cup_name] = XY(t.x, t.y)

            except Exception:
                return None

        return cups

    # ------------------------------------------------------
    # Receive latest UDP block packet only
    # ------------------------------------------------------
    def get_latest_blocks_udp(self):
        latest_data = None

        while True:
            try:
                data, _ = self.udp_sock.recvfrom(4096)
                latest_data = data
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"[UDP] Error: {e}")
                return None

        if latest_data is None:
            return None

        try:
            block_list = json.loads(latest_data.decode())
        except Exception as e:
            self.get_logger().warn(f"[UDP] JSON error: {e}")
            return None

        blocks = {}

        for block in block_list:
            marker_id = int(block["id"])

            # WARNING:
            # Right now cx/cy are pixels, not meters.
            # This lets the code run, but the real solver needs x/y in base_link.
            name = f"block_{marker_id}"

            blocks[name] = Block(
                name=name,
                x=float(block["cx"]),
                y=float(block["cy"]),
                color="unknown"
            )

        return blocks

    # ------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------
    def state_machine(self):
        cups = self.get_cups()
        if cups is None:
            return

        blocks = self.get_latest_blocks_udp()
        if not blocks:
            return

        best = compute_best_pickup(cups, blocks)

        if best is None:
            self.get_logger().info("[SOLVER] No valid pickup found")
            return

        self.get_logger().info(
            f"[SOLVER] BEST | "
            f"pick={best.picked_count} | "
            f"score={best.score:.2f} | "
            f"error={best.avg_error:.3f} | "
            f"yaw={math.degrees(best.yaw):.1f} deg | "
            f"dx={best.dx:.3f}, dy={best.dy:.3f} | "
            f"assignments={best.assignments}"
        )


# ==========================================================
# MAIN
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MergedLocalPickupNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()