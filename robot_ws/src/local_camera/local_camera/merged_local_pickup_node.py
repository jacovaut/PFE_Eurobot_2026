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
import rclpy.duration
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from collections import defaultdict
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass
class XY:
    x: float
    y: float

class MergedLocalPickupNode(Node):
    def __init__(self):
        super().__init__('merged_local_pickup_node')
        # Parameters
        self.declare_parameter('debug_save_image', False)
        self.debug_save_image = bool(self.get_parameter('debug_save_image').value)
        self.declare_parameter('camera_mode', 'stream')
        self.declare_parameter('stream_url', 'tcp://127.0.0.1:8888')
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('show_debug_window', False)
        self.camera_mode = str(self.get_parameter('camera_mode').value).strip().lower()
        self.stream_url = str(self.get_parameter('stream_url').value).strip()
        self.cameraDeviceNumber = int(self.get_parameter('camera_device').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.output_width = 1280
        self.output_height = 720
        # Camera
        self.camera = None
        self.state = 'init_camera'
        self.state_timer = self.create_timer(0.2, self.state_machine)
        # Perception
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'topic_camera_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Pickup
        self.cup_frames = ["cup_0", "cup_1", "cup_2", "cup_3"]
        self.required_cups = len(self.cup_frames)
        self.memory = {}
        self.memory_timeout = 1.0
        self.next_track_index = {}
        self.match_dist = 0.12
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
        self.id_color_map = {47: "jaune", 36: "bleu"}
        self.cups_detected = 0
        self.blocks_detected = 0
        self.cups_ready = False
        self.blocks_ready = False
        self.solver_ready = False
        # ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.03
        self.camera_matrix = np.array([
            [457.33917579,   0.0,         637.592287  ],
            [  0.0,         453.81772548, 374.90978642],
            [  0.0,           0.0,           1.        ] 
        ], dtype=np.float64)
        self.dist_coeffs = np.array([
            -0.0241479, 
            -0.01872201,  
            0.00181977, 
            -0.00044101,  
            0.04127062
        ], dtype=np.float64)
        self.get_logger().info("[MERGED] Node initialized. Starting state machine.")

    def publish_cup_tf(self, cup_index, tvec, q_cam, stamp=None):
        # Publish TF for cup_N in base_link or arducam_optical_frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() if stamp is None else stamp
        t.header.frame_id = "arducam_optical_frame"  # or "base_link" if you prefer
        t.child_frame_id = f"cup_{cup_index}"
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = float(q_cam[0])
        t.transform.rotation.y = float(q_cam[1])
        t.transform.rotation.z = float(q_cam[2])
        t.transform.rotation.w = float(q_cam[3])
        self.tf_broadcaster.sendTransform(t)

    def state_machine(self):
        # 1. Ensure camera is connected
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().info('[SEQ] Connecting to camera...')
            self.camera = cv2.VideoCapture(self.stream_url if self.camera_mode == 'stream' else self.cameraDeviceNumber)
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.output_width)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.output_height)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
                self.get_logger().info('[SEQ] Camera connected.')
            else:
                self.get_logger().warn('[SEQ] Camera not ready, retrying...')
                return

        # 2. Try to lookup all cup frames in TF
        cups = {}
        for cup_name in self.cup_frames:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', cup_name, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                t = tf.transform.translation
                cups[cup_name] = XY(t.x, t.y)
            except Exception:
                self.get_logger().info(f'[SEQ] Waiting for TF: {cup_name}')
                return
        self.get_logger().info(f'[SEQ] All cup frames found in TF.')

        # 3. Detect blocks with vision
        success, frame = self.camera.read()
        if not success or frame is None:
            self.get_logger().warn('[SEQ] No frame from camera, waiting...')
            return
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, self.dictionary, parameters=self.parameters)
        blocks = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # Use all detected markers as blocks
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_points_blc,
                    np.array(corners[i], dtype=np.float32),
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not ok:
                    continue
                tvec = tvec.reshape(3)
                blocks[f'block_{int(marker_id)}'] = XY(tvec[0], tvec[1])
        self.get_logger().info(f'[SEQ] Blocks detected: {len(blocks)}')
        if len(blocks) == 0:
            self.get_logger().info('[SEQ] Waiting for blocks...')
            return

        # 4. Run pickup calculation (placeholder)
        self.get_logger().info('[SEQ] Running pickup calculation!')
        # Here you would call your compute_best_alignment() logic
        self.solver_ready = True
        self.get_logger().info('[SEQ] Pickup calculation complete. (Placeholder)')

def main(args=None):
    rclpy.init(args=args)
    node = MergedLocalPickupNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            if node.camera is not None:
                node.camera.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
