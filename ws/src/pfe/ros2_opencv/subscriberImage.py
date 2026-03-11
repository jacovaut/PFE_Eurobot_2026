#!/usr/bin/env python3
import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import json
import time
import tf_transformations
from tf2_ros import TransformBroadcaster

# ===========================================================
# CLASSE : CaisseNoisette (track stable per instance)
# ===========================================================
class CaisseNoisette:
    def __init__(self, marker_id: int, index: int, x: float, y: float, z: float, quat, color: str):
        self.id = int(marker_id)
        self.index = int(index)  # stable-ish per track
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.quat = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))  # (x,y,z,w)
        self.color = color
        self.last_seen = time.time()

    def update(self, x: float, y: float, z: float, quat):
        alpha = 0.30
        self.x = self.x * (1 - alpha) + float(x) * alpha
        self.y = self.y * (1 - alpha) + float(y) * alpha
        self.z = self.z * (1 - alpha) + float(z) * alpha

        # Blend quaternion a bit + normalize (good enough for visualization)
        q_old = np.array(self.quat, dtype=float)
        q_new = np.array(quat, dtype=float)
        q = q_old * (1 - alpha) + q_new * alpha
        n = np.linalg.norm(q)
        if n > 1e-9:
            q = q / n
        self.quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

        self.last_seen = time.time()

    def to_dict(self):
        return {
            "id": self.id,
            "index": self.index,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "color": self.color
        }


# ===========================================================
# NODE
# ===========================================================
class SubscriberNodeClass(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # --- ROS2 setup ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'topic_camera_image', self.listener_callback, 20
        )
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- ArUco setup ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.05  # meters

        # --- Camera calibration (placeholder) ---
        self.camera_matrix = np.array([[600.0, 0.0, 320.0],
                                       [0.0, 600.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))

        # --- Map ID -> couleur ---
        self.id_color_map = {47: "jaune", 36: "bleu"}  # edit as needed

        # --- Tracking memory ---
        # key = (marker_id, track_index) -> CaisseNoisette
        self.memory = {}
        self.memory_timeout = 1.0  # seconds
        self.next_track_index = {}  # marker_id -> next new index

        # --- Association threshold (meters) ---
        # If blocks are close together, reduce this (e.g., 0.06-0.10).
        self.match_dist = 0.12

        self.get_logger().info("✅ Subscriber node with multi-same-ID tracking + RViz markers started.")

    def _associate_track(self, marker_id: int, x: float, y: float, z: float, claimed_keys: set):
        """
        Find closest existing track of same marker_id within match_dist,
        that is NOT already claimed by another detection in this same frame.
        Returns (track_key, track_obj) or (None, None).
        """
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

    def listener_callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        current_time = time.time()

        # Track claiming set (prevents 4 -> 3 merge)
        claimed_keys = set()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                mid = int(marker_id)
                if mid not in self.id_color_map:
                    continue

                color = self.id_color_map[mid]
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])

                # --- Proper quaternion from rvec ---
                R, _ = cv2.Rodrigues(rvec)

                # Extract yaw from rotation matrix (camera frame)
                yaw = float(np.arctan2(R[1, 0], R[0, 0]))

                # Make quaternion that only rotates around Z (no roll/pitch)
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

                # --- Associate to existing track (unique per frame) ---
                track_key, track = self._associate_track(mid, x, y, z, claimed_keys)

                if track is not None:
                    track.update(x, y, z, quat)
                    claimed_keys.add(track_key)
                else:
                    # allocate new track index (can increase forever; OK)
                    idx = self.next_track_index.get(mid, 0)
                    self.next_track_index[mid] = idx + 1

                    track = CaisseNoisette(mid, idx, x, y, z, quat, color)
                    new_key = (mid, idx)
                    self.memory[new_key] = track
                    claimed_keys.add(new_key)

                # --- TF publish: unique frame per tracked instance ---
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
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

                # Optional debug axes
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        # --- Clean memory ---
        to_delete = [k for k, c in self.memory.items() if current_time - c.last_seen > self.memory_timeout]
        for k in to_delete:
            del self.memory[k]

        # --- MarkerArray for RViz ---
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for c in self.memory.values():
            marker = Marker()
            marker.header.frame_id = "arducam_optical_frame"
            marker.header.stamp = stamp

            # Unique marker identity in RViz is (ns, id)
            marker.ns = f"aruco_{c.id}"
            marker.id = int(c.index)

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = c.x
            marker.pose.position.y = c.y
            marker.pose.position.z = c.z

            marker.pose.orientation.x = c.quat[0]
            marker.pose.orientation.y = c.quat[1]
            marker.pose.orientation.z = c.quat[2]
            marker.pose.orientation.w = c.quat[3]

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

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

        # --- JSON output ---
        msg = String()
        msg.data = json.dumps([c.to_dict() for c in self.memory.values()])
        self.pub_blocks.publish(msg)

        # --- Camera debug window ---
        cv2.imshow("Camera Video with Tracking", frame)
        cv2.waitKey(1)


# ===========================================================
# MAIN
# ===========================================================
def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNodeClass()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()