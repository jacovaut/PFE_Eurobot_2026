import rclpy
from rclpy.node import Node
import rclpy.duration
import socket
import json
import math
import numpy as np
import tf_transformations
from geometry_msgs.msg import Pose2D, TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .modules.data_structures import XY, Block, PickupCandidate
from .modules.math_utils import rotate_xy, angle_between, wrap_angle, rectangular_yaw_diff_deg
from .modules.solver import compute_best_pickup, recompute_locked_pose
from .modules.block_tracker import BlockTracker
from .modules.tf_utils import TfPublisher

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
        self.id_to_color = {36: "blue", 47: "yellow"}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]
        self.block_tracker = BlockTracker(self.block_timeout_sec, self.match_distance_m)
        self.tf_publisher = TfPublisher(self.tf_broadcaster, self.get_clock)
        self.lock_required_frames = 5
        self.reset_lock()
        self.locked_targets = {}
        self.current_cups = {}
        self.pickup_pose_pub = self.create_publisher(Pose2D, "/pickup_pose", 10)
        self.best_pickup_pub = self.create_publisher(String, "/best_pickup", 10)
        self.manip_info_pub = self.create_publisher(String, "/manip_info", 10)
        self.pickup_status_sub = self.create_subscription(String, "/pickup_status", self.pickup_status_callback, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.udp_port))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("[SOLVER READY]")

    def reset_lock(self):
        self.solution_locked = False
        self.locked_signature = None
        self.locked_best = None
        self.locked_targets = {}
        self.candidate_signature = None
        self.candidate_count = 0
        self.block_targets = {"cup_0": None, "cup_1": None, "cup_2": None, "cup_3": None}

    def pickup_status_callback(self, msg):
        status = msg.data.lower().strip()
        if status in ["done", "unlock", "success", "failed", "aligned"]:
            self.reset_lock()

    def get_signature(self, best):
        return "|".join(sorted(f"{a['cup']}->{a['block']}" for a in best.assignments))

    def update_lock(self, best):
        if self.solution_locked:
            frozen_blocks = {}
            for a in self.locked_best.assignments:
                block_name = a["block"]
                if block_name in self.block_tracker.tracked_blocks:
                    frozen_blocks[block_name] = self.block_tracker.tracked_blocks[block_name]
                elif block_name in self.locked_targets:
                    frozen_blocks[block_name] = self.locked_targets[block_name]
            updated = recompute_locked_pose(self.current_cups, frozen_blocks, self.locked_best.assignments)
            if updated:
                self.locked_best.dx = updated.dx
                self.locked_best.dy = updated.dy
                self.locked_best.yaw = updated.yaw
                self.locked_best.avg_error = updated.avg_error
                self.locked_best.assignments = updated.assignments
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
            self.locked_targets = {}
            for a in best.assignments:
                block_name = a["block"]
                if block_name in self.block_tracker.tracked_blocks:
                    self.locked_targets[block_name] = self.block_tracker.tracked_blocks[block_name]
            self.get_logger().info(f"[LOCKED] {sig}")
            return best
        self.get_logger().info(f"[CANDIDATE] {sig} ({self.candidate_count}/{self.lock_required_frames})")
        return None

    def loop(self):
        cups = {}
        for name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform("base_link", name, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.005))
                t = tf.transform.translation
                cups[name] = XY(t.x, t.y)
            except Exception:
                pass
        self.current_cups = cups
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
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "arducam_optical_frame", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.005))
        except Exception:
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
                p = np.array([float(b["x_cam"]), float(b["y_cam"]), float(b["z_cam"]), 1.0])
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
            except Exception:
                pass
        now = self.get_clock().now().nanoseconds * 1e-9
        locked_names = set()
        if self.solution_locked and self.locked_best:
            for a in self.locked_best.assignments:
                locked_names.add(a["block"])
        blocks = self.block_tracker.update_tracking(detections, now, locked_names=locked_names)
        self.tf_publisher.publish_block_transforms(self.block_targets)
        if not blocks:
            return
        best = compute_best_pickup(cups, blocks, self.team_color)
        locked = self.update_lock(best)
        if not locked:
            return
        self.publish_best_pickup(locked)

    def publish_best_pickup(self, best):
        for key in self.block_targets:
            self.block_targets[key] = None
        for a in best.assignments:
            cup_name = a["cup"]
            block_name = a["block"]
            if block_name in self.locked_targets:
                self.block_targets[cup_name] = self.locked_targets[block_name]
            elif block_name in self.block_tracker.tracked_blocks:
                self.block_targets[cup_name] = self.block_tracker.tracked_blocks[block_name]
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
        color_map = {"blue": "B", "yellow": "Y"}
        for a in best.assignments:
            pump[a["cup"]] = color_map.get(a["color"], "")
        manip = String()
        manip.data = (
            f"P1={pump['cup_0']} ,"
            f"P2={pump['cup_1']} ,"
            f"P3={pump['cup_2']} ,"
            f"P4={pump['cup_3']}"
        )
        self.manip_info_pub.publish(manip)

def main():
    rclpy.init()
    node = MergedLocalPickupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
