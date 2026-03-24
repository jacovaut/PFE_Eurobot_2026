#!/usr/bin/env python3
import os
import shutil
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CsiCameraPublisherNode(Node):
    def __init__(self):
        super().__init__('csi_camera_publisher_node')

        self.declare_parameter('topic_name', 'topic_camera_image')
        self.declare_parameter('frame_id', 'arducam_optical_frame')
        self.declare_parameter('image_encoding', 'mono8')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_command', '')
        self.declare_parameter(
            'ld_preload',
            '/usr/local/lib/aarch64-linux-gnu/libcamera.so.0.7:'
            '/usr/local/lib/aarch64-linux-gnu/libcamera-base.so.0.7:'
            '/usr/local/lib/aarch64-linux-gnu/libpisp.so.1',
        )
        self.declare_parameter('ld_library_path', '/usr/local/lib/aarch64-linux-gnu')
        self.declare_parameter('chunk_size', 65536)
        self.declare_parameter('restart_delay', 1.0)

        self.topic_name = str(self.get_parameter('topic_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.image_encoding = str(self.get_parameter('image_encoding').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.framerate = int(self.get_parameter('framerate').value)
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.camera_command = str(self.get_parameter('camera_command').value).strip()
        self.ld_preload = str(self.get_parameter('ld_preload').value).strip()
        self.ld_library_path = str(self.get_parameter('ld_library_path').value).strip()
        self.chunk_size = int(self.get_parameter('chunk_size').value)
        self.restart_delay = float(self.get_parameter('restart_delay').value)

        if self.image_encoding not in {'mono8', 'bgr8'}:
            raise ValueError("image_encoding must be 'mono8' or 'bgr8'")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.topic_name, 10)

        self._stop_event = threading.Event()
        self._process: Optional[subprocess.Popen] = None
        self._worker_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker_thread.start()

        self.get_logger().info(
            f"CSI publisher started on topic '{self.topic_name}' using {self.image_encoding} frames"
        )

    def _resolve_camera_command(self) -> str:
        if self.camera_command:
            if os.path.isfile(self.camera_command) and os.access(self.camera_command, os.X_OK):
                return self.camera_command
            resolved = shutil.which(self.camera_command)
            if resolved:
                return resolved
            raise RuntimeError(f"Camera command '{self.camera_command}' was not found")

        candidates = [
            '/usr/local/bin/rpicam-vid',
            '/usr/bin/rpicam-vid',
            '/usr/local/bin/libcamera-vid',
            '/usr/bin/libcamera-vid',
        ]
        for candidate in candidates:
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate

        raise RuntimeError('No CSI camera command found. Install rpicam-vid or libcamera-vid.')

    def _build_command(self) -> list[str]:
        command = self._resolve_camera_command()
        return [
            command,
            '--nopreview',
            '--timeout',
            '0',
            '--width',
            str(self.width),
            '--height',
            str(self.height),
            '--framerate',
            str(self.framerate),
            '--camera',
            str(self.camera_index),
            '--codec',
            'mjpeg',
            '--output',
            '-',
        ]

    def _build_environment(self) -> dict[str, str]:
        env = os.environ.copy()
        if self.ld_preload:
            env['LD_PRELOAD'] = self.ld_preload
        if self.ld_library_path:
            current = env.get('LD_LIBRARY_PATH', '')
            env['LD_LIBRARY_PATH'] = (
                f"{self.ld_library_path}:{current}" if current else self.ld_library_path
            )
        return env

    @staticmethod
    def _decode_frame(jpeg_bytes: bytes):
        jpeg_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        return cv2.imdecode(jpeg_array, cv2.IMREAD_COLOR)

    def _publish_frame(self, frame):
        if self.image_encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.image_encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _capture_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self._run_capture_session()
            except Exception as exc:
                self.get_logger().error(f'CSI capture failed: {exc}')

            if self._stop_event.is_set():
                break

            time.sleep(self.restart_delay)
            self.get_logger().warn('Restarting CSI camera process')

    def _run_capture_session(self):
        command = self._build_command()
        env = self._build_environment()

        self.get_logger().info(f"Starting CSI command: {' '.join(command)}")

        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
            env=env,
        )

        if self._process.stdout is None:
            raise RuntimeError('CSI camera process has no stdout pipe')

        buffer = bytearray()

        try:
            while rclpy.ok() and not self._stop_event.is_set():
                chunk = self._process.stdout.read(self.chunk_size)
                if not chunk:
                    return_code = self._process.poll()
                    raise RuntimeError(f'CSI camera process stopped with code {return_code}')

                buffer.extend(chunk)

                while True:
                    start = buffer.find(b'\xff\xd8')
                    if start < 0:
                        if len(buffer) > self.chunk_size:
                            del buffer[:-2]
                        break

                    end = buffer.find(b'\xff\xd9', start + 2)
                    if end < 0:
                        if start > 0:
                            del buffer[:start]
                        break

                    jpeg_bytes = bytes(buffer[start:end + 2])
                    del buffer[:end + 2]

                    frame = self._decode_frame(jpeg_bytes)
                    if frame is None:
                        self.get_logger().warn('Failed to decode MJPEG frame from CSI process')
                        continue

                    self._publish_frame(frame)
        finally:
            self._terminate_process()

    def _terminate_process(self):
        process = self._process
        self._process = None
        if process is None:
            return

        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

        if process.stdout is not None:
            process.stdout.close()

    def destroy_node(self):
        self._stop_event.set()
        self._terminate_process()
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsiCameraPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()