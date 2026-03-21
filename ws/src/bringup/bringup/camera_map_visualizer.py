#!/usr/bin/env python3

import json
import math
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class CameraMapVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('camera_map_visualizer_node')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_pose_topic = self.declare_parameter('robot_pose_topic', '/camera/global_pose').value
        self.detected_blocks_topic = self.declare_parameter('detected_blocks_topic', '/detected_blocks').value
        self.marker_topic = self.declare_parameter('marker_topic', '/camera_map/markers').value
        self.block_pointcloud_topic = self.declare_parameter('block_pointcloud_topic', '/camera/block_obstacles').value
        self.publish_block_obstacles = bool(self.declare_parameter('publish_block_obstacles', True).value)
        self.max_blocks_visualized = int(self.declare_parameter('max_blocks_visualized', 32).value)
        self.block_obstacle_height_m = float(self.declare_parameter('block_obstacle_height_m', 0.10).value)

        self.nids_noms = list(self.declare_parameter('nids_noms', ['nid_jaune', 'nid_bleu']).value)
        self.nids_centre_x_m = list(self.declare_parameter('nids_centre_x_m', [0.300, 2.700]).value)
        self.nids_centre_y_m = list(self.declare_parameter('nids_centre_y_m', [1.775, 1.775]).value)
        self.nids_taille_x_m = list(self.declare_parameter('nids_taille_x_m', [0.600, 0.600]).value)
        self.nids_taille_y_m = list(self.declare_parameter('nids_taille_y_m', [0.450, 0.450]).value)

        self.garde_manger_noms = list(self.declare_parameter('garde_manger_noms', []).value)
        self.garde_manger_centre_x_m = list(self.declare_parameter('garde_manger_centre_x_m', []).value)
        self.garde_manger_centre_y_m = list(self.declare_parameter('garde_manger_centre_y_m', []).value)
        self.garde_manger_taille_x_m = list(self.declare_parameter('garde_manger_taille_x_m', []).value)
        self.garde_manger_taille_y_m = list(self.declare_parameter('garde_manger_taille_y_m', []).value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.block_pc_pub = self.create_publisher(PointCloud2, self.block_pointcloud_topic, 10)

        self.create_subscription(PoseWithCovarianceStamped, self.robot_pose_topic, self._robot_pose_cb, 20)
        self.create_subscription(String, self.detected_blocks_topic, self._blocks_cb, 20)

        self.get_logger().info('camera_map_visualizer_node started')

    def _robot_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.map_frame
        t.child_frame_id = 'camera_robot'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _blocks_cb(self, msg: String) -> None:
        blocks = self._parse_blocks(msg.data)

        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = now
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker_id = 0
        marker_id = self._append_zone_markers(
            marker_array,
            now,
            marker_id,
            ns='nids',
            names=self.nids_noms,
            xs=self.nids_centre_x_m,
            ys=self.nids_centre_y_m,
            size_xs=self.nids_taille_x_m,
            size_ys=self.nids_taille_y_m,
            color=(1.0, 1.0, 1.0, 0.22),
        )
        marker_id = self._append_zone_markers(
            marker_array,
            now,
            marker_id,
            ns='garde_manger',
            names=self.garde_manger_noms,
            xs=self.garde_manger_centre_x_m,
            ys=self.garde_manger_centre_y_m,
            size_xs=self.garde_manger_taille_x_m,
            size_ys=self.garde_manger_taille_y_m,
            color=(0.3, 0.9, 0.3, 0.18),
        )

        obstacle_points: List[List[float]] = []

        for i, block in enumerate(blocks[: self.max_blocks_visualized]):
            child_name = f'block_{i:02d}'
            self._publish_block_tf(now, child_name, block)

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now
            marker.ns = 'detected_blocks'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(block.get('x', 0.0))
            marker.pose.position.y = float(block.get('y', 0.0))
            marker.pose.position.z = float(block.get('z', 0.0))
            self._set_quaternion_from_yaw(marker, float(block.get('yaw', 0.0)))
            marker.scale.x = max(float(block.get('size_x_m', 0.12)), 0.03)
            marker.scale.y = max(float(block.get('size_y_m', 0.05)), 0.03)
            marker.scale.z = max(self.block_obstacle_height_m, 0.03)
            self._set_color_for_block(marker, str(block.get('color', 'unknown')).lower())
            marker_array.markers.append(marker)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = 'detected_blocks_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = marker.pose.position.x
            text.pose.position.y = marker.pose.position.y
            text.pose.position.z = marker.pose.position.z + 0.12
            text.scale.z = 0.07
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{block.get('color', 'unk')} #{i}"
            marker_array.markers.append(text)

            obstacle_points.append([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ])

        self.marker_pub.publish(marker_array)

        if self.publish_block_obstacles:
            header = Header()
            header.stamp = now
            header.frame_id = self.map_frame
            cloud = point_cloud2.create_cloud_xyz32(header, obstacle_points)
            self.block_pc_pub.publish(cloud)

    def _append_zone_markers(
        self,
        marker_array: MarkerArray,
        stamp,
        start_id: int,
        ns: str,
        names: List[str],
        xs: List[float],
        ys: List[float],
        size_xs: List[float],
        size_ys: List[float],
        color,
    ) -> int:
        count = min(len(names), len(xs), len(ys), len(size_xs), len(size_ys))
        marker_id = start_id
        for i in range(count):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(xs[i])
            marker.pose.position.y = float(ys[i])
            marker.pose.position.z = 0.005
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(size_xs[i])
            marker.scale.y = float(size_ys[i])
            marker.scale.z = 0.01
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker_array.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = f'{ns}_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(xs[i])
            label.pose.position.y = float(ys[i])
            label.pose.position.z = 0.06
            label.scale.z = 0.05
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = names[i]
            marker_array.markers.append(label)
        return marker_id

    def _publish_block_tf(self, stamp, child_name: str, block: Dict[str, Any]) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = child_name
        t.transform.translation.x = float(block.get('x', 0.0))
        t.transform.translation.y = float(block.get('y', 0.0))
        t.transform.translation.z = float(block.get('z', 0.0))

        yaw = float(block.get('yaw', 0.0))
        half = yaw * 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform(t)

    def _parse_blocks(self, raw_json: str) -> List[Dict[str, Any]]:
        try:
            data = json.loads(raw_json)
            if isinstance(data, list):
                return [item for item in data if isinstance(item, dict) and item.get('color') != 'robot']
        except json.JSONDecodeError:
            self.get_logger().warning('Failed to parse detected_blocks JSON')
        return []

    @staticmethod
    def _set_quaternion_from_yaw(marker: Marker, yaw: float) -> None:
        half = yaw * 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(half)
        marker.pose.orientation.w = math.cos(half)

    @staticmethod
    def _set_color_for_block(marker: Marker, color_name: str) -> None:
        marker.color.a = 0.85
        if color_name == 'jaune':
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.1
        elif color_name == 'bleu':
            marker.color.r = 0.1
            marker.color.g = 0.4
            marker.color.b = 1.0
        else:
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraMapVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
