#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import rclpy.duration
import socket
import json
from dataclasses import dataclass


@dataclass
class XY:
    x: float
    y: float


class MergedLocalPickupNode(Node):
    def __init__(self):
        super().__init__('merged_local_pickup_node')

        # Parameters
        self.declare_parameter('udp_port', 5005)
        self.udp_port = int(self.get_parameter('udp_port').value)

        # Run at 20 Hz
        self.timer = self.create_timer(0.05, self.state_machine)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Cups
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]

        # UDP socket (non-blocking)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.udp_sock.bind(("0.0.0.0", self.udp_port))
        self.udp_sock.setblocking(False)

        self.get_logger().info("[MERGED] Ready (low-latency mode)")

    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def state_machine(self):

        # ------------------------------------------------------
        # 1. Get cup TFs (fast)
        # ------------------------------------------------------
        cups = {}

        for cup_name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link',
                    cup_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.005)  # FAST
                )

                t = tf.transform.translation
                cups[cup_name] = XY(t.x, t.y)

            except Exception:
                return  # wait until all cups exist

        # ------------------------------------------------------
        # 2. Receive UDP (drain buffer → keep latest)
        # ------------------------------------------------------
        latest_data = None

        while True:
            try:
                data, _ = self.udp_sock.recvfrom(4096)
                latest_data = data
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"UDP error: {e}")
                return

        if latest_data is None:
            return

        # ------------------------------------------------------
        # 3. Decode blocks
        # ------------------------------------------------------
        try:
            block_list = json.loads(latest_data.decode())
        except:
            return

        blocks = {}

        for block in block_list:
            blocks[f"block_{block['id']}"] = XY(
                block['cx'],
                block['cy']
            )

        if not blocks:
            return

        # ------------------------------------------------------
        # 4. RUN SOLVER (placeholder)
        # ------------------------------------------------------
        self.get_logger().info(f"[SOLVER] {len(blocks)} blocks, computing...")

        # 👉 Replace this with your real solver
        # compute_best_alignment(cups, blocks)

        # Debug print
        # print(cups, blocks)


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


if __name__ == '__main__':
    main()