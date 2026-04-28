#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import json

class BlockPublisher(Node):
    def __init__(self):
        super().__init__('block_publisher')
        self.publisher_ = self.create_publisher(String, 'block_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # Use your camera device

        # ArUco setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No frame from camera')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        blocks = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))
                blocks.append({'id': int(marker_id), 'cx': cx, 'cy': cy})

        msg = String()
        msg.data = json.dumps(blocks)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BlockPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()