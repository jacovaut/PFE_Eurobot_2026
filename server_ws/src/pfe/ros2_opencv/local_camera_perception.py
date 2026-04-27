#!/usr/bin/env python3
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
import rclpy.duration

from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class LocalCameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('local_camera_perception_node')

        # ===========================================================
        # CAMERA STREAM (FROM HOST rpicam-vid)
        # ===========================================================
        self.stream_url = "tcp://127.0.0.1:8888"

        self.get_logger().info(f"Opening stream: {self.stream_url}")
        self.camera = cv2.VideoCapture(self.stream_url)

        if not self.camera.isOpened():
            self.get_logger().error("❌ Could not open TCP camera stream")
            raise RuntimeError("Camera stream failed")

        # Fix resize bug
        self.output_width = 1280
        self.output_height = 720

        # ===========================================================
        # ROS
        # ===========================================================
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'topic_camera_image', 10)
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===========================================================
        # ARUCO
        # ===========================================================
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.marker_length = 0.03

        self.camera_matrix = np.array([
            [457.33917579, 0.0, 637.592287],
            [0.0, 453.81772548, 374.90978642],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.dist_coeffs = np.array([
            -0.0241479,
            -0.01872201,
            0.00181977,
            -0.00044101,
            0.04127062
        ], dtype=np.float64)

        m = self.marker_length / 2.0
        self.obj_points_blc = np.array([
            [-m, m, 0.0],
            [m, m, 0.0],
            [m, -m, 0.0],
            [-m, -m, 0.0],
        ], dtype=np.float32)

        self.id_color_map = {
            47: "jaune",
            36: "bleu"
        }

        self.memory = {}
        self.memory_timeout = 1.0
        self.next_track_index = {}
        self.match_dist = 0.12

        self.block_size = 0.05
        self.block_center_z = self.block_size / 2.0

        self.pickup_frame_parent = "base_link"
        self.pickup_frame_name = "pickup_frame"

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("✅ Perception node running (TCP stream mode)")

    # ===========================================================
    # MAIN LOOP
    # ===========================================================
    def timer_callback(self):
        ret, frame = self.camera.read()

        if not ret or frame is None:
            self.get_logger().warn("No frame from TCP stream")
            return

        frame = cv2.resize(frame, (self.output_width, self.output_height))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        stamp = self.get_clock().now().to_msg()

        if ids is not None:
            self.get_logger().info(f"Detected markers: {ids.flatten()}")

            for i, marker_id in enumerate(ids.flatten()):
                mid = int(marker_id)

                if mid not in self.id_color_map:
                    continue

                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_points_blc,
                    np.array(corners[i], dtype=np.float32),
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not ok:
                    continue

                x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])

                # Publish simple TF directly (TEMP debug)
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = "base_link"
                t.child_frame_id = f"block_{mid}_{i}"

                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z

                t.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(t)

                self.get_logger().info(f"Published block_{mid}_{i}")

    # ===========================================================
    # MAIN
    # ===========================================================
def main():
    rclpy.init()
    node = LocalCameraPerceptionNode()
    rclpy.spin(node)
    node.camera.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()  