#!/usr/bin/env python3
import os
import cv2
import rclpy
import math
import time
import numpy as np
import json
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations
import rclpy.duration
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from collections import defaultdict


# ===========================================================
# TRACKED BLOCK CLASS
# ===========================================================
class CaisseNoisette:
    def __init__(self, marker_id: int, index: int, x: float, y: float, z: float, quat, color: str):
        self.id = int(marker_id)
        self.index = int(index)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.quat = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        self.color = color
        self.last_seen = time.time()
        self.pickup_x = None
        self.pickup_y = None
        self.pickup_z = None
        self.pickup_quat = None

    def update(self, x: float, y: float, z: float, quat):
        alpha = 0.30
        self.x = self.x * (1 - alpha) + float(x) * alpha
        self.y = self.y * (1 - alpha) + float(y) * alpha
        self.z = self.z * (1 - alpha) + float(z) * alpha
        q_old = np.array(self.quat, dtype=float)
        q_new = np.array(quat, dtype=float)
        q = q_old * (1 - alpha) + q_new * alpha
        n = np.linalg.norm(q)
        if n > 1e-9:
            q = q / n
        self.quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self.last_seen = time.time()

    def update_pickup_pose(self, x: float, y: float, z: float, quat):
        alpha = 0.30
        if self.pickup_x is None:
            self.pickup_x = float(x)
            self.pickup_y = float(y)
            self.pickup_z = float(z)
            self.pickup_quat = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
            return
        self.pickup_x = self.pickup_x * (1 - alpha) + float(x) * alpha
        self.pickup_y = self.pickup_y * (1 - alpha) + float(y) * alpha
        self.pickup_z = self.pickup_z * (1 - alpha) + float(z) * alpha
        q_old = np.array(self.pickup_quat, dtype=float)
        q_new = np.array(quat, dtype=float)
        q = q_old * (1 - alpha) + q_new * alpha
        n = np.linalg.norm(q)
        if n > 1e-9:
            q = q / n
        self.pickup_quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def to_dict(self):
        data = {
            "id": self.id,
            "index": self.index,
            "x_cam": self.x,
            "y_cam": self.y,
            "z_cam": self.z,
            "color": self.color
        }
        if self.pickup_x is not None:
            yaw = None
            if self.pickup_quat is not None:
                _, _, yaw = tf_transformations.euler_from_quaternion(self.pickup_quat)
            data.update({"x": self.pickup_x, "y": self.pickup_y, "z": self.pickup_z, "yaw": yaw})
        return data


# ===========================================================
# XY DATACLASS
# ===========================================================
@dataclass
class XY:
    x: float
    y: float


# ===========================================================
# MERGED NODE
# ===========================================================
class MergedLocalPickupNode(Node):
    def __init__(self):
        super().__init__('merged_local_pickup_node')

        # ---------- Camera parameters ----------
        self.declare_parameter('camera_device', -1)
        self.declare_parameter('camera_path', 'libcamera')
        self.declare_parameter('show_debug_window', True)
        self.cameraDeviceNumber = int(self.get_parameter('camera_device').value)
        self.camera_path = str(self.get_parameter('camera_path').value).strip()
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        camera_source = self.camera_path if self.camera_path else self.cameraDeviceNumber

        if self.camera_path == "libcamera":
            pipeline = (
                "libcamerasrc ! "
                "video/x-raw,width=1280,height=720,framerate=30/1 ! "
                "videoconvert ! "
                "appsink"
            )
            self.camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self.camera = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
            if not self.camera.isOpened():
                self.camera = cv2.VideoCapture(camera_source)

        if not self.camera.isOpened():
            self.get_logger().error(f"Could not open camera source {camera_source}")
            raise RuntimeError("Camera open failed")

        self.get_logger().info(f"Using camera source: {camera_source}")
        self.get_logger().info(f"Debug preview window enabled: {self.show_debug_window}")
        self.output_width = 1280
        self.output_height = 720

        # ---------- ROS publishers / subscribers ----------
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'topic_camera_image', 10)
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- Shared TF buffer ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- ArUco ----------
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.03  # meters
        self.camera_matrix = np.array([
            [457.33917579,   0.0,         637.592287  ],
            [  0.0,         453.81772548, 374.90978642],
            [  0.0,           0.0,           1.        ]
        ], dtype=np.float64)
        self.dist_coeffs = np.array([
            -0.0241479, -0.01872201, 0.00181977, -0.00044101, 0.04127062
        ], dtype=np.float64)
        self.get_logger().info("Using HARDCODED onboard camera calibration (1080p)")
        m = self.marker_length / 2.0
        self.obj_points_blc = np.array([
            [-m,  m, 0.0], [ m,  m, 0.0], [ m, -m, 0.0], [-m, -m, 0.0],
        ], dtype=np.float32)
        self.id_color_map = {47: "jaune", 36: "bleu"}

        # ---------- Tracking ----------
        self.memory = {}
        self.memory_timeout = 1.0
        self.next_track_index = {}
        self.match_dist = 0.12

        # ---------- Block plane ----------
        self.block_size = 0.05
        self.block_center_z = self.block_size / 2.0
        self.pickup_frame_parent = "base_link"
        self.pickup_frame_name = "pickup_frame"
        self.pickup_tx = 0.0
        self.pickup_ty = 0.0
        self.pickup_tz = 0.0
        self.pickup_roll = 0.0
        self.pickup_pitch = 0.0
        self.pickup_yaw = 0.0

        # ---------- Pickup solver ----------
        self.ref_frame = "base_link"
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]
        self.team_color = "yellow"
        self.BLUE_IDS = {36}
        self.YELLOW_IDS = {47}
        self.match_radius_m = 0.015
        self.pickup_max_err_m = 0.015
        self.min_useful_matches = 2
        self.pickup_min_matches = 2
        self.tf_timeout = Duration(seconds=0.03)
        self.block_memory_s = 0.50
        self.block_last_seen: Dict[str, float] = {}
        self.max_blocks_to_consider = 20
        self.missing_block_cycles_allowed = 10
        self.max_local_dx_m = 0.40
        self.max_local_dy_m = 0.25
        self.max_local_dyaw_deg = 95.0
        self.yaw_min_deg = -95.0
        self.yaw_max_deg = 95.0
        self.yaw_step_deg = 1.0
        self.w_matches = 500.0
        self.w_color = 10000.0
        self.w_error = 1.0
        self.w_yaw = 0.3
        self.w_translation = 80.0
        self.w_row = 400.0
        self.row_max_perp_error_m = 0.01
        self.w_consecutive = 250.0
        self.required_stable_cycles = 7
        self.stable_dx_tol_m = 0.01
        self.stable_dy_tol_m = 0.01
        self.stable_dyaw_tol_deg = 3.0
        self.prev_solution = None
        self.prev_assignment_signature = None
        self.stable_cycles = 0
        self.block_history: Dict[str, List[XY]] = {}
        self.block_history_len = 10
        self.use_smoothed_blocks = True
        self.motion_reset_threshold_m = 0.03
        self.debug_candidates = True
        self.debug_top_k = 10
        self.debug_tf_children = False
        self.debug_geometry = False
        self.debug_block_stability = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(TFMessage, "/tf", self.tf_cb, qos)
        self.pickup_goal_pub = self.create_publisher(String, "pickup_goal", 10)
        self.block_queue_pub = self.create_publisher(String, "block_queue_cmd", 10)
        self.pickup_state_sub = self.create_subscription(
            String, "pickup_state", self.pickup_state_cb, 10
        )
        self.pickup_cups_pub = self.create_publisher(String, "pickup_cups_cmd", 10)
        self.locked = False
        self.locked_signature = None
        self.locked_assignments = None
        self.pickup_state = "idle"
        self.pickup_timer = None

        # ---------- Timers ----------
        self.create_timer(0.1, self.timer_callback)   # 10 Hz - camera loop
        self.create_timer(0.2, self.tick)              # 5 Hz  - pickup solver

        self.get_logger().info("Merged local pickup node started.")

    # =========================================================
    # Camera / Perception methods
    # =========================================================

    def _associate_track(self, marker_id: int, x: float, y: float, z: float, claimed_keys: set):
        best_key = None
        best = None
        best_d2 = 1e18
        for key, c in self.memory.items():
            if c.id != marker_id:
                continue
            if key in claimed_keys:
                continue
            d2 = (c.x - x) ** 2 + (c.y - y) ** 2 + (c.z - z) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = c
                best_key = key
        if best is not None and best_d2 <= (self.match_dist ** 2):
            return best_key, best
        return None, None

    def _publish_pickup_frame(self, stamp):
        q = tf_transformations.quaternion_from_euler(
            self.pickup_roll, self.pickup_pitch, self.pickup_yaw
        )
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.pickup_frame_parent
        t.child_frame_id = self.pickup_frame_name
        t.transform.translation.x = self.pickup_tx
        t.transform.translation.y = self.pickup_ty
        t.transform.translation.z = self.pickup_tz
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_raw_aruco_tf(self, track, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "arducam_optical_frame"
        t.child_frame_id = f"aruco_{track.id}_{track.index}"
        t.transform.translation.x = track.x
        t.transform.translation.y = track.y
        t.transform.translation.z = track.z
        t.transform.rotation.x = track.quat[0]
        t.transform.rotation.y = track.quat[1]
        t.transform.rotation.z = track.quat[2]
        t.transform.rotation.w = track.quat[3]
        self.tf_broadcaster.sendTransform(t)

    def _compute_and_publish_clean_block_tf(self, track, stamp):
        self.get_logger().debug(f"compute_clean_tf: track id={track.id} idx={track.index}")
        try:
            ref_frame = "base_link"
            camera_frame = "arducam_optical_frame"
            can = self.tf_buffer.can_transform(
                ref_frame, camera_frame,
                self.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            if not can:
                self.get_logger().warn(
                    "TF not ready: base_link -> arducam_optical_frame (skipping block TF)"
                )
                return
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                ref_frame, camera_frame,
                self.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            t = tf_cam_to_base.transform.translation
            q_tf = tf_cam_to_base.transform.rotation
            trans = np.array([t.x, t.y, t.z], dtype=float)
            quat_tf = np.array([q_tf.x, q_tf.y, q_tf.z, q_tf.w], dtype=float)
            T = tf_transformations.quaternion_matrix(quat_tf)
            T[0:3, 3] = trans
            p_cam = np.array([track.x, track.y, track.z, 1.0], dtype=float)
            p_base = (T @ p_cam)[:3]
            quat_cam = np.array(track.quat, dtype=float)
            quat_base_raw = tf_transformations.quaternion_multiply(quat_tf, quat_cam)
            _, _, yaw = tf_transformations.euler_from_quaternion(quat_base_raw)
            q_flat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
            track.update_pickup_pose(float(p_base[0]), float(p_base[1]), float(self.block_center_z), q_flat)
            tb = TransformStamped()
            tb.header.stamp = stamp
            tb.header.frame_id = ref_frame
            tb.child_frame_id = f"block_{track.id}_{track.index}"
            tb.transform.translation.x = track.pickup_x
            tb.transform.translation.y = track.pickup_y
            tb.transform.translation.z = track.pickup_z
            tb.transform.rotation.x = track.pickup_quat[0]
            tb.transform.rotation.y = track.pickup_quat[1]
            tb.transform.rotation.z = track.pickup_quat[2]
            tb.transform.rotation.w = track.pickup_quat[3]
            self.tf_broadcaster.sendTransform(tb)
            self.get_logger().info(
                f"Published block TF: block_{track.id}_{track.index} "
                f"@ ({track.pickup_x:.3f}, {track.pickup_y:.3f}, {track.pickup_z:.3f})"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish block TF for block_{track.id}_{track.index}: {e}"
            )

    def timer_callback(self):
        success, frame = self.camera.read()
        if not success or frame is None:
            self.get_logger().warn("No frame captured from camera")
            return

        frame = cv2.resize(frame, (self.output_width, self.output_height), interpolation=cv2.INTER_CUBIC)

        if frame.ndim == 2:
            gray_frame = frame
            display_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
        elif frame.ndim == 3 and frame.shape[2] == 1:
            gray_frame = frame[:, :, 0]
            display_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
        elif frame.ndim == 3 and frame.shape[2] >= 3:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            display_frame = frame
        else:
            self.get_logger().warn(f"Unsupported frame shape: {frame.shape}")
            return

        ros_img = self.bridge.cv2_to_imgmsg(gray_frame, encoding="mono8")
        self.image_pub.publish(ros_img)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray_frame, self.dictionary, parameters=self.parameters
        )

        current_time = time.time()
        claimed_keys = set()
        stamp = self.get_clock().now().to_msg()
        self._publish_pickup_frame(stamp)

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected {len(ids)} ArUco markers: {ids.flatten()}")
            cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                mid = int(marker_id)
                if mid not in self.id_color_map:
                    continue
                color = self.id_color_map[mid]

                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_points_blc,
                    np.array(corners[i], dtype=np.float32),
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not ok:
                    continue

                rvec = rvec.reshape(3)
                tvec = tvec.reshape(3)
                x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])

                R, _ = cv2.Rodrigues(rvec)
                T_rot = np.eye(4)
                T_rot[:3, :3] = R
                q_cam = tf_transformations.quaternion_from_matrix(T_rot)
                quat_cam = (float(q_cam[0]), float(q_cam[1]), float(q_cam[2]), float(q_cam[3]))

                track_key, track = self._associate_track(mid, x, y, z, claimed_keys)
                if track is not None:
                    track.update(x, y, z, quat_cam)
                    claimed_keys.add(track_key)
                else:
                    idx = self.next_track_index.get(mid, 0)
                    self.next_track_index[mid] = idx + 1
                    track = CaisseNoisette(mid, idx, x, y, z, quat_cam, color)
                    new_key = (mid, idx)
                    self.memory[new_key] = track
                    claimed_keys.add(new_key)

                self._publish_raw_aruco_tf(track, stamp)
                self._compute_and_publish_clean_block_tf(track, stamp)

                cv2.drawFrameAxes(display_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

                label = f"{mid}:{track.index}"
                corner = corners[i][0][0]
                cv2.putText(
                    display_frame, label,
                    (int(corner[0]), int(corner[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
                )

        to_delete = [k for k, c in self.memory.items() if current_time - c.last_seen > self.memory_timeout]
        for k in to_delete:
            del self.memory[k]

        marker_array = MarkerArray()
        for c in self.memory.values():
            if c.pickup_x is None or c.pickup_quat is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.pickup_frame_name
            marker.header.stamp = stamp
            marker.ns = f"block_{c.id}"
            marker.id = int(c.index)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = c.pickup_x
            marker.pose.position.y = c.pickup_y
            marker.pose.position.z = c.pickup_z
            marker.pose.orientation.x = c.pickup_quat[0]
            marker.pose.orientation.y = c.pickup_quat[1]
            marker.pose.orientation.z = c.pickup_quat[2]
            marker.pose.orientation.w = c.pickup_quat[3]
            marker.scale.x = self.block_size
            marker.scale.y = self.block_size
            marker.scale.z = self.block_size
            if c.color == "jaune":
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            elif c.color == "bleu":
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0
            marker.color.a = 0.9
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        msg = String()
        msg.data = json.dumps([c.to_dict() for c in self.memory.values()])
        self.pub_blocks.publish(msg)

        if self.show_debug_window:
            cv2.imshow("Merged Camera + Tracking", display_frame)
            cv2.waitKey(1)

    # =========================================================
    # Pickup solver methods
    # =========================================================

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
                self.ref_frame, target_frame,
                rclpy.time.Time(), timeout=self.tf_timeout,
            )
            tr = tf.transform.translation
            return XY(tr.x, tr.y)
        except Exception:
            return None

    @staticmethod
    def rotate_xy(p: XY, yaw_rad: float) -> XY:
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return XY(x=c * p.x - s * p.y, y=s * p.x + c * p.y)

    def debug_pairwise_spacing(self, label: str, items: Dict[str, XY], sort_axis: str = "y"):
        if len(items) == 0:
            return
        ordered = sorted(items.items(), key=lambda item: item[1].x if sort_axis == "x" else item[1].y)
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
                self.get_logger().info(f"{n1} -> {n2}: dx={dx:+.3f}, dy={dy:+.3f}, dist={d*1000:.1f} mm")

    def compute_row_bonus(self, assignments: List[Tuple[str, str, float]], blocks: Dict[str, XY]) -> float:
        if len(assignments) < 2:
            return 0.0
        matched_points = [blocks[blk] for _, blk, _ in assignments if blk in blocks]
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
                ux, uy = dx / norm, dy / norm
                perp_errors = [abs((p.x - p1.x) * uy - (p.y - p1.y) * ux) for p in matched_points]
                avg_perp = sum(perp_errors) / len(perp_errors)
                if avg_perp < best_avg_perp:
                    best_avg_perp = avg_perp
        if best_avg_perp == float("inf") or best_avg_perp >= self.row_max_perp_error_m:
            return 0.0
        return self.w_row * (1.0 - best_avg_perp / self.row_max_perp_error_m)

    def compute_consecutive_bonus(self, assignments: List[Tuple[str, str, float]]) -> float:
        if len(assignments) < 2:
            return 0.0
        cup_indices = []
        for cup_name, _, _ in assignments:
            try:
                cup_indices.append(int(cup_name.split("_")[1]))
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
            marker_id = int(frame_name.split("_")[1])
        except Exception:
            return "unknown"
        if marker_id in self.BLUE_IDS:
            return "blue"
        elif marker_id in self.YELLOW_IDS:
            return "yellow"
        return "unknown"

    def color_value(self, frame_name: str) -> float:
        color = self.get_block_color(frame_name)
        if color == "unknown":
            return 0.0
        if self.team_color == "yellow":
            return 1.0 if color == "blue" else 0.2 if color == "yellow" else 0.0
        elif self.team_color == "blue":
            return 1.0 if color == "yellow" else 0.2 if color == "blue" else 0.0
        return 0.0

    def update_block_history(self, blocks: Dict[str, XY]):
        for name, xy in blocks.items():
            if name not in self.block_history:
                self.block_history[name] = [xy]
                continue
            last = self.block_history[name][-1]
            if math.hypot(xy.x - last.x, xy.y - last.y) > self.motion_reset_threshold_m:
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
            else:
                smoothed[name] = XY(
                    sum(p.x for p in history) / len(history),
                    sum(p.y for p in history) / len(history),
                )
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
            if nearest_block is not None and nearest_dist <= self.match_radius_m:
                matches += 1
                total_err_m += nearest_dist
                color_score += self.color_value(nearest_block)
                assignments.append((cup_name, nearest_block, nearest_dist))
                del unmatched_blocks[nearest_block]
        avg_err_mm = (total_err_m / matches) * 1000.0 if matches > 0 else 1e9
        weighted_score = (
            self.w_matches * matches
            + self.w_color * color_score
            + self.compute_row_bonus(assignments, blocks_dict)
            + self.compute_consecutive_bonus(assignments)
            - self.w_error * avg_err_mm
            - self.w_yaw * abs(math.degrees(yaw_rad))
            - self.w_translation * math.hypot(dx, dy)
        )
        return matches, weighted_score, assignments, avg_err_mm, color_score

    def generate_candidates(self, cups: Dict[str, XY], blocks: Dict[str, XY]) -> List[Dict]:
        candidates = []
        block_list = list(blocks.items())
        yaw_deg = self.yaw_min_deg
        while yaw_deg <= self.yaw_max_deg + 1e-9:
            yaw_rad = math.radians(yaw_deg)
            rotated_cups = [(name, self.rotate_xy(p, yaw_rad)) for name, p in cups.items()]
            for _, cup_xy in rotated_cups:
                for _, block_xy in block_list:
                    dx = block_xy.x - cup_xy.x
                    dy = block_xy.y - cup_xy.y
                    if abs(dx) > self.max_local_dx_m or abs(dy) > self.max_local_dy_m:
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
            yaw_deg += self.yaw_step_deg
        return candidates

    def select_best_candidate(self, candidates: List[Dict]) -> Dict:
        if not candidates:
            return {}
        return max(candidates, key=lambda c: (c["matches"], c["score"]))

    def deduplicate_candidates(self, candidates: List[Dict]) -> List[Dict]:
        best_per_assignment: Dict = defaultdict(lambda: {"score": -1e18, "cand": None})
        for cand in candidates:
            assign_sig = tuple(sorted((cup, blk) for cup, blk, _ in cand["assignments"]))
            if cand["score"] > best_per_assignment[assign_sig]["score"]:
                best_per_assignment[assign_sig] = {"score": cand["score"], "cand": cand}
        return [v["cand"] for v in best_per_assignment.values() if v["cand"] is not None]

    def compute_best_alignment(self, cups: Dict[str, XY], blocks: Dict[str, XY]):
        candidates = self.generate_candidates(cups, blocks)
        candidates = self.deduplicate_candidates(candidates)
        candidates.sort(key=lambda c: (c["matches"], c["score"]), reverse=True)
        best_cand = self.select_best_candidate(candidates)
        if not best_cand:
            return (0.0, 0.0, 0.0, 0, -1e18, [], 1e9, 0.0)
        if self.debug_candidates and candidates:
            self.log_candidates(candidates)
        return (
            math.radians(best_cand["yaw_deg"]), best_cand["dx"], best_cand["dy"],
            best_cand["matches"], best_cand["score"], best_cand["assignments"],
            best_cand["avg_err_mm"], best_cand["color_score"]
        )

    def log_candidates(self, candidates: List[Dict]):
        four_match = [c for c in candidates if c["matches"] == 4]
        self.get_logger().info(f"debug | num_4_match_candidates={len(four_match)}")
        self.get_logger().info("===== TOP CANDIDATES =====")
        for i, cand in enumerate(candidates[:self.debug_top_k]):
            assign_str = ", ".join(
                [f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in cand["assignments"]]
            )
            self.get_logger().info(
                f"[{i}] matches={cand['matches']} | score={cand['score']:.1f} | ... | {assign_str}"
            )
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
            if self.locked and self.locked_assignments is not None:
                self.publish_block_queue(self.locked_assignments)
                self.publish_pickup_cups(self.locked_assignments)
                self.get_logger().info("PUBLISHED block queue and cup assignments after arrived")
        elif s == "done":
            self.get_logger().info("pickup_state: done")
            self.pickup_state = "done"
            self.locked = False
            self.locked_signature = None
            self.locked_assignments = None
        elif s == "reset":
            self.get_logger().info("pickup_state: reset")
            self.locked = False
            self.pickup_state = "idle"
            self.locked_signature = None
            self.locked_assignments = None

    def publish_block_queue(self, assignments: List[Tuple[str, str, float]]):
        ordered = sorted(assignments, key=lambda a: int(a[0].split("_")[1]), reverse=True)
        msg = String()
        msg.data = ",".join(self.get_block_color(blk) for _, blk, _ in ordered)
        self.block_queue_pub.publish(msg)

    def publish_pickup_cups(self, assignments: List[Tuple[str, str, float]]):
        ordered = sorted(assignments, key=lambda a: int(a[0].split("_")[1]), reverse=True)
        msg = String()
        msg.data = ",".join(cup.split("_")[1] for cup, _, _ in ordered)
        self.pickup_cups_pub.publish(msg)

    def publish_pickup_goal(self, dx, dy, yaw_deg, assignments):
        plan = {
            "dx": dx, "dy": dy, "yaw_deg": yaw_deg,
            "assignments": [
                {"cup": cup, "block": blk, "color": self.get_block_color(blk)}
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
                self.get_logger().info(f"waiting for cups... got {len(cups)}/{len(self.cup_frames)}")
                self.stable_cycles = 0
                self.prev_solution = None
                self.prev_assignment_signature = None
                return

            raw_blocks: Dict[str, XY] = {}
            for b in self.get_recent_block_frames():
                xy = self.lookup_xy(b)
                if xy is not None:
                    raw_blocks[b] = xy

            self.update_block_history(raw_blocks)
            self.print_block_stability()
            blocks = self.get_smoothed_blocks(raw_blocks)

            if self.locked:
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
                    self.get_logger().info(f"{name} ({self.get_block_color(name)}): x={xy.x:+.3f}, y={xy.y:+.3f}")
                self.debug_pairwise_spacing("CUPS", cups, sort_axis="y")
                self.debug_pairwise_spacing("BLOCKS", blocks, sort_axis="y")

            self.get_logger().info("debug | entering compute_best_alignment()")
            (
                best_yaw, best_dx, best_dy,
                best_matches, best_score, best_assignments,
                best_avg_err_mm, best_color_score,
            ) = self.compute_best_alignment(cups, blocks)
            self.get_logger().info("debug | compute_best_alignment() returned")
            self.get_logger().info(
                f"debug | matches={best_matches} | score={best_score:.1f} | "
                f"color_score={best_color_score:.2f} | avg_err={best_avg_err_mm:.1f} mm"
            )

            if best_matches < self.min_useful_matches:
                self.get_logger().info(f"No worthwhile pickup found. best_matches={best_matches}")
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

            yaw_deg = math.degrees(best_yaw)
            assignment_signature = tuple(sorted([blk for _, blk, _ in best_assignments]))

            if assignment_signature != self.prev_assignment_signature:
                self.stable_cycles = 1
                self.prev_solution = (best_dx, best_dy, yaw_deg)
                self.prev_assignment_signature = assignment_signature
            else:
                self.update_stability(best_dx, best_dy, best_yaw)

            ready = self.is_pickup_ready(best_matches, best_assignments)

            if ready and not self.locked:
                self.locked = True
                self.locked_signature = assignment_signature
                self.locked_assignments = best_assignments
                self.pickup_state = "moving"
                self.publish_pickup_goal(best_dx, best_dy, yaw_deg, best_assignments)
                self.get_logger().info("PUBLISHED pickup goal, locking until done")
                locked_assign_str = ", ".join(
                    [f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in best_assignments]
                )
                self.get_logger().info(f"Locked solution blocks: {locked_assign_str}")

            if self.locked:
                return

            dt_ms = (time.time() - start_t) * 1000.0
            assign_str = ", ".join(
                [f"{cup}->{blk} ({dist*1000:.1f} mm)" for cup, blk, dist in best_assignments]
            )
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


# ===========================================================
# MAIN
# ===========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MergedLocalPickupNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.camera.release()
        except Exception:
            pass
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
