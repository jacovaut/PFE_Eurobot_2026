#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import json
import time
import tf_transformations
import rclpy.duration  # add this import


from tf2_ros import TransformBroadcaster, Buffer, TransformListener


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
        self.quat = (
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3])
        )  # raw optical-frame quaternion
        self.color = color
        self.last_seen = time.time()

        # cleaned pose in pickup_frame
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
            self.pickup_quat = (
                float(quat[0]),
                float(quat[1]),
                float(quat[2]),
                float(quat[3])
            )
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
            data.update({
                "x": self.pickup_x,
                "y": self.pickup_y,
                "z": self.pickup_z,
                "yaw": yaw
            })

        return data


# ===========================================================
# MERGED CAMERA + PERCEPTION NODE
# ===========================================================
class LocalCameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('local_camera_perception_node')

        # ---------- Camera ----------
        self.declare_parameter('camera_device', 4) # Place proper camera device number here (check with v4l2-ctl --list-devices)
        self.declare_parameter('camera_path', '')
        self.declare_parameter('show_debug_window', True)

        self.cameraDeviceNumber = int(self.get_parameter('camera_device').value)
        self.camera_path = str(self.get_parameter('camera_path').value).strip()
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)

        camera_source = self.camera_path if self.camera_path else self.cameraDeviceNumber
        self.camera = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.camera = cv2.VideoCapture(camera_source)

        if not self.camera.isOpened():
            self.get_logger().error(f"Could not open camera source {camera_source}")
            raise RuntimeError("Camera open failed")

        self.get_logger().info(f"Using camera source: {camera_source}")
        self.get_logger().info(f"Debug preview window enabled: {self.show_debug_window}")

        # Keep this consistent with your current setup
        self.output_width = 1280
        self.output_height = 720

        # ---------- ROS ----------
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'topic_camera_image', 10)
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- ArUco ----------
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

       # ===========================================================
        # CAMERA CALIBRATION (HARDCODED - 1920x1080)
        # ===========================================================

        # Marker size (IMPORTANT: 30 mm)
        self.marker_length = 0.03  # meters

        # Camera intrinsic matrix (from calibration)
        self.camera_matrix = np.array([
            [457.33917579,   0.0,         637.592287  ],
            [  0.0,         453.81772548, 374.90978642],
            [  0.0,           0.0,           1.        ] 
        ], dtype=np.float64)
 
        # Distortion coefficients
        self.dist_coeffs = np.array([
            -0.0241479, 
            -0.01872201,  
            0.00181977, 
            -0.00044101,  
            0.04127062
        ], dtype=np.float64)

        self.get_logger().info("✅ Using HARDCODED onboard camera calibration (1080p)")

        self.camera_matrix = np.array(self.camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.array(self.dist_coeffs, dtype=np.float64)

        self.get_logger().info(f"Camera matrix:\n{self.camera_matrix}")
        self.get_logger().info(f"Dist coeffs:\n{self.dist_coeffs}")

        # Square object points for solvePnP IPPE_SQUARE
        m = self.marker_length / 2.0
        self.obj_points_blc = np.array([
            [-m,  m, 0.0],
            [ m,  m, 0.0],
            [ m, -m, 0.0],
            [-m, -m, 0.0],
        ], dtype=np.float32)

        # ---------- Marker ID -> color ----------
        self.id_color_map = {
            47: "jaune",
            36: "bleu"
        }

        # ---------- Tracking ----------
        self.memory = {}
        self.memory_timeout = 1.0
        self.next_track_index = {}
        self.match_dist = 0.12

        # ---------- Flattened block plane ----------
        self.block_size = 0.05
        self.block_center_z = self.block_size / 2.0

        # Temporary flat frame relative to base_link
        self.pickup_frame_parent = "base_link"
        self.pickup_frame_name = "pickup_frame"

        self.pickup_tx = 0.0
        self.pickup_ty = 0.0
        self.pickup_tz = 0.0

        # Tune these later if needed
        self.pickup_roll = 0.0
        self.pickup_pitch = 0.0
        self.pickup_yaw = 0.0

        # ---------- Timer ----------
        self.periodCommunication = 0.1  # 10 Hz
        self.timer = self.create_timer(self.periodCommunication, self.timer_callback)

        self.get_logger().info("✅ Merged camera + ArUco perception node started.")

    # -------------------------------------------------------------------------
    # Tracking helper
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # TF helpers
    # -------------------------------------------------------------------------
    def _publish_pickup_frame(self, stamp):
        q = tf_transformations.quaternion_from_euler(
            self.pickup_roll,
            self.pickup_pitch,
            self.pickup_yaw
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

    def _intersect_ray_with_pickup_plane(self, ray_origin, ray_point, plane_z=0.0):
        d = ray_point - ray_origin
        if abs(d[2]) < 1e-9:
            return None

        s = (plane_z - ray_origin[2]) / d[2]
        if s < 0.0:
            return None

        return ray_origin + s * d

    def _compute_and_publish_clean_block_tf(self, track, stamp):
        self.get_logger().debug(f"compute_clean_tf: track id={track.id} idx={track.index}")

        try:
            ref_frame = "base_link"
            camera_frame = "arducam_optical_frame"

            can = self.tf_buffer.can_transform(
                ref_frame,
                camera_frame,
                self.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            if not can:
                self.get_logger().warn(
                    "TF not ready: base_link -> arducam_optical_frame (skipping block TF)"
                )
                return

            tf_cam_to_base = self.tf_buffer.lookup_transform(
                ref_frame,
                camera_frame,
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
            p_base_h = T @ p_cam
            p_base = p_base_h[:3]

            cam_origin_base = trans

            # Project the detected block position onto the pickup plane (flat table).
            # We keep X/Y from the computed base-frame position and force Z to the table plane.
            p_proj = np.array([p_base[0], p_base[1], 0.0], dtype=float)

            

            quat_cam = np.array(track.quat, dtype=float)
            quat_base_raw = tf_transformations.quaternion_multiply(quat_tf, quat_cam)

            _, _, yaw = tf_transformations.euler_from_quaternion(quat_base_raw)
            q_flat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

            track.update_pickup_pose(
                float(p_proj[0]),
                float(p_proj[1]),
                float(self.block_center_z),
                q_flat
            )

            tb = TransformStamped()
            tb.header.stamp = stamp
            tb.header.frame_id = ref_frame                # publish in base_link
            tb.child_frame_id = f"block_{track.id}_{track.index}"
            tb.transform.translation.x = track.pickup_x
            tb.transform.translation.y = track.pickup_y
            tb.transform.translation.z = track.pickup_z
            tb.transform.rotation.x = track.pickup_quat[0]
            tb.transform.rotation.y = track.pickup_quat[1]
            tb.transform.rotation.z = track.pickup_quat[2]
            tb.transform.rotation.w = track.pickup_quat[3]
            self.tf_broadcaster.sendTransform(tb)
            self.get_logger().info(f"Published block TF: block_{track.id}_{track.index} "
                                   f"@ ({track.pickup_x:.3f}, {track.pickup_y:.3f}, {track.pickup_z:.3f})")
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish block TF for block_{track.id}_{track.index}: {e}"
            )

    # -------------------------------------------------------------------------
    # Main camera loop
    # -------------------------------------------------------------------------
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
            self.get_logger().warn(f"Unsupported frame shape from camera: {frame.shape}")
            return

        # Optional image publish
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

                # Explicit PnP like your external camera
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

                # Full quaternion from rvec
                R, _ = cv2.Rodrigues(rvec)
                T_rot = np.eye(4)
                T_rot[:3, :3] = R
                q_cam = tf_transformations.quaternion_from_matrix(T_rot)
                quat_cam = (
                    float(q_cam[0]),
                    float(q_cam[1]),
                    float(q_cam[2]),
                    float(q_cam[3])
                )

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

                cv2.drawFrameAxes(
                    display_frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.03
                )

                # Draw marker id:index on the image
                label = f"{mid}:{track.index}"
                corner = corners[i][0][0]  # top-left corner of the marker
                pos = (int(corner[0]), int(corner[1]) - 10)  # slightly above marker
                cv2.putText(
                    display_frame,
                    label,
                    pos,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        # Clean memory
        to_delete = [k for k, c in self.memory.items() if current_time - c.last_seen > self.memory_timeout]
        for k in to_delete:
            del self.memory[k]

        # Debug block loss
        lost_blocks = [k for k, c in self.memory.items() if current_time - c.last_seen > self.memory_timeout]
        if lost_blocks:
            self.get_logger().info(f"Lost {len(lost_blocks)} blocks due to timeout: {lost_blocks}")

        # RViz markers in pickup_frame
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
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif c.color == "bleu":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0

            marker.color.a = 0.9
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        # JSON output
        msg = String()
        msg.data = json.dumps([c.to_dict() for c in self.memory.values()])
        self.pub_blocks.publish(msg)

        # Debug view
        if self.show_debug_window:
            cv2.imshow("Merged Camera + Tracking", display_frame)
            cv2.waitKey(1)


# ===========================================================
# MAIN
# ===========================================================
def main(args=None):
    rclpy.init(args=args)
    node = LocalCameraPerceptionNode()
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