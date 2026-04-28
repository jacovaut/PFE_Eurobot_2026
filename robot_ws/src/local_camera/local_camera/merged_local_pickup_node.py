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
        self.declare_parameter('udp_port', 5005)
        self.udp_port = int(self.get_parameter('udp_port').value)
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
        # Add missing object points for solvePnP (marker corners in 3D)
        m = self.marker_length / 2.0
        self.obj_points_blc = np.array([
            [-m,  m, 0.0],
            [ m,  m, 0.0],
            [ m, -m, 0.0],
            [-m, -m, 0.0],
        ], dtype=np.float32)

        # Optional: Debug image saving
        self.debug_image_path = '/tmp/merged_node_debug.jpg'
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

        # 1. (Camera logic removed, not used)

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


        # 3. Receive blocks via UDP
        import socket
        import json
        UDP_IP = "0.0.0.0"
        UDP_PORT = self.udp_port
        if not hasattr(self, 'udp_sock'):
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.bind((UDP_IP, UDP_PORT))
            self.udp_sock.setblocking(False)

        blocks = {}
        try:
            data, addr = self.udp_sock.recvfrom(4096)
            block_list = json.loads(data.decode())
            for block in block_list:
                blocks[f"block_{block['id']}"] = XY(block['cx'], block['cy'])
            self.get_logger().info(f'[SEQ][UDP] Blocks received: {len(blocks)}')
        except BlockingIOError:
            self.get_logger().info('[SEQ][UDP] No block info received yet...')
            return
        except Exception as e:
            self.get_logger().warn(f'[SEQ][UDP] Error receiving block info: {e}')
            return
        if len(blocks) == 0:
            self.get_logger().info('[SEQ][UDP] Waiting for blocks...')
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
