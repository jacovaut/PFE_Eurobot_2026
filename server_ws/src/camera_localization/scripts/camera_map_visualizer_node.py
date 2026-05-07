#!/usr/bin/env python3

import json
import faulthandler
import math
import os
import sys
import traceback
from typing import Any, Dict, List

STARTUP_LOG = '/tmp/camera_map_visualizer_node_startup.log'


def _startup_log(message: str) -> None:
    try:
        with open(STARTUP_LOG, 'a', encoding='utf-8') as handle:
            handle.write(message + '\n')
            handle.flush()
    except Exception:
        pass


try:
    _startup_log(
        f'process imported: pid={os.getpid()} executable={sys.executable} argv={sys.argv}'
    )
    faulthandler.enable(file=sys.stderr, all_threads=True)
except Exception:
    pass

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.detected_enemy_topic = self.declare_parameter('detected_enemy_topic', '/detected_enemy').value
        self.marker_topic = self.declare_parameter('marker_topic', '/camera_map/markers').value
        self.block_pointcloud_topic = self.declare_parameter('block_pointcloud_topic', '/camera/block_obstacles').value
        self.static_pointcloud_topic = self.declare_parameter('static_pointcloud_topic', '/camera/static_obstacles').value
        self.enemy_pointcloud_topic = self.declare_parameter('enemy_pointcloud_topic', '/camera/enemy_obstacles').value
        self.dynamic_obstacle_grid_topic = self.declare_parameter(
            'dynamic_obstacle_grid_topic',
            '/camera/block_obstacle_grid',
        ).value
        self.enemy_obstacle_grid_topic = self.declare_parameter(
            'enemy_obstacle_grid_topic',
            '/camera/enemy_obstacle_grid',
        ).value
        self.publish_block_obstacles = bool(self.declare_parameter('publish_block_obstacles', True).value)
        self.max_blocks_visualized = int(self.declare_parameter('max_blocks_visualized', 32).value)
        self.block_obstacle_height_m = float(self.declare_parameter('block_obstacle_height_m', 0.03).value)
        self.detected_obstacle_point_spacing_m = float(
            self.declare_parameter('detected_obstacle_point_spacing_m', 0.05).value
        )
        self.detected_obstacle_padding_m = float(
            self.declare_parameter('detected_obstacle_padding_m', 0.0).value
        )
        self.enemy_robot_obstacle_padding_m = float(
            self.declare_parameter('enemy_robot_obstacle_padding_m', 0.0).value
        )
        self.publish_period_s = float(self.declare_parameter('publish_period_s', 0.25).value)
        self.dynamic_obstacle_stale_timeout_s = float(
            self.declare_parameter('dynamic_obstacle_stale_timeout_s', 0.75).value
        )
        self.dynamic_obstacle_max_miss_count = int(
            self.declare_parameter('dynamic_obstacle_max_miss_count', 10).value
        )
        self.enemy_stale_timeout_s = float(
            self.declare_parameter('enemy_stale_timeout_s', 1.5).value
        )
        self.enemy_max_miss_count = int(
            self.declare_parameter('enemy_max_miss_count', 3).value
        )
        self.dynamic_obstacle_match_distance_m = float(
            self.declare_parameter('dynamic_obstacle_match_distance_m', 0.20).value
        )
        self.stamp_obstacle_cloud_with_now = bool(
            self.declare_parameter('stamp_obstacle_cloud_with_now', False).value
        )

        self.nids_noms = list(self.declare_parameter('nids_noms', ['nid_jaune', 'nid_bleu']).value)
        self.nids_centre_x_m = list(self.declare_parameter('nids_centre_x_m', [0.300, 2.700]).value)
        self.nids_centre_y_m = list(self.declare_parameter('nids_centre_y_m', [1.775, 1.775]).value)
        self.nids_taille_x_m = list(self.declare_parameter('nids_taille_x_m', [0.600, 0.600]).value)
        self.nids_taille_y_m = list(self.declare_parameter('nids_taille_y_m', [0.450, 0.450]).value)

        self.garde_manger_noms = self._declare_string_array_parameter('garde_manger_noms')
        self.garde_manger_centre_x_m = self._declare_double_array_parameter('garde_manger_centre_x_m')
        self.garde_manger_centre_y_m = self._declare_double_array_parameter('garde_manger_centre_y_m')
        self.garde_manger_taille_x_m = self._declare_double_array_parameter('garde_manger_taille_x_m')
        self.garde_manger_taille_y_m = self._declare_double_array_parameter('garde_manger_taille_y_m')

        self.zones_interdites_noms = self._declare_string_array_parameter('zones_interdites_noms')
        self.zones_interdites_centre_x_m = self._declare_double_array_parameter('zones_interdites_centre_x_m')
        self.zones_interdites_centre_y_m = self._declare_double_array_parameter('zones_interdites_centre_y_m')
        self.zones_interdites_taille_x_m = self._declare_double_array_parameter('zones_interdites_taille_x_m')
        self.zones_interdites_taille_y_m = self._declare_double_array_parameter('zones_interdites_taille_y_m')
        self.zones_interdites_point_spacing_m = float(
            self.declare_parameter('zones_interdites_point_spacing_m', 0.05).value
        )
        self.static_obstacle_padding_m = float(
            self.declare_parameter('static_obstacle_padding_m', 0.0).value
        )
        self.dynamic_obstacle_grid_resolution_m = float(
            self.declare_parameter('dynamic_obstacle_grid_resolution_m', 0.05).value
        )

        self.arena_x_min = float(self.declare_parameter('arena_x_min', 0.0).value)
        self.arena_x_max = float(self.declare_parameter('arena_x_max', 3.0).value)
        self.arena_y_min = float(self.declare_parameter('arena_y_min', 0.0).value)
        self.arena_y_max = float(self.declare_parameter('arena_y_max', 2.0).value)
        self.arena_wall_spacing_m = float(self.declare_parameter('arena_wall_spacing_m', 0.05).value)
        self.arena_wall_z_m = float(self.declare_parameter('arena_wall_z_m', 0.05).value)
        self.publish_arena_walls_as_obstacles = bool(
            self.declare_parameter('publish_arena_walls_as_obstacles', True).value
        )

        self._wall_points: List[List[float]] = self._build_wall_points()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.block_pc_pub = self.create_publisher(PointCloud2, self.block_pointcloud_topic, 10)
        self.static_pc_pub = self.create_publisher(PointCloud2, self.static_pointcloud_topic, 10)
        self.enemy_pc_pub = self.create_publisher(PointCloud2, self.enemy_pointcloud_topic, 10)
        dynamic_grid_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.dynamic_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.dynamic_obstacle_grid_topic,
            dynamic_grid_qos,
        )
        self.enemy_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.enemy_obstacle_grid_topic,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.latest_blocks_time = self.get_clock().now()
        self.latest_enemy_time = self.get_clock().now()
        self.tracked_blocks: List[Dict[str, Any]] = []
        self.tracked_enemy: List[Dict[str, Any]] = []
        self.has_received_blocks = False
        self.has_received_enemy = False
        self.has_received_robot_pose = False

        self.create_subscription(PoseWithCovarianceStamped, self.robot_pose_topic, self._robot_pose_cb, 20)
        self.create_subscription(String, self.detected_blocks_topic, self._blocks_cb, 20)
        self.create_subscription(String, self.detected_enemy_topic, self._enemy_cb, 20)
        self.create_timer(max(self.publish_period_s, 0.05), self._publish_visualization)

        self.get_logger().info('camera_map_visualizer_node started')

    def _robot_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.has_received_robot_pose = True
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
        detections = self._parse_blocks(msg.data)
        self.latest_blocks_time = self.get_clock().now()
        blocks_only = [b for b in detections if str(b.get('color', '')).lower() != 'enemy_robot']
        self.tracked_blocks = self._update_tracked_obstacles(blocks_only, self.tracked_blocks, self.dynamic_obstacle_max_miss_count)
        self.has_received_blocks = True

    def _enemy_cb(self, msg: String) -> None:
        detections = self._parse_blocks(msg.data)
        self.latest_enemy_time = self.get_clock().now()
        if detections:
            # Singleton: overwrite with the latest position from C++ (which already
            # holds last-known when not detecting, so we always trust what C++ sends).
            self.tracked_enemy = [{'block': detections[0], 'miss_count': 0}]
        # If empty: C++ has no track yet (enemy never seen) — keep existing.
        self.has_received_enemy = True

    def _publish_visualization(self) -> None:
        now_time = self.get_clock().now()
        blocks_age_s = (now_time - self.latest_blocks_time).nanoseconds * 1e-9
        enemy_age_s = (now_time - self.latest_enemy_time).nanoseconds * 1e-9
        blocks_are_stale = (
            self.has_received_blocks
            and blocks_age_s > max(self.dynamic_obstacle_stale_timeout_s, 0.0)
        )
        if blocks_are_stale:
            self.get_logger().warn(
                f'Block obstacle data is stale ({blocks_age_s:.2f}s old); clearing blocks',
                throttle_duration_sec=1.0,
            )
        blocks_only = (
            self._active_tracked_obstacles(self.tracked_blocks, now_time, self.dynamic_obstacle_stale_timeout_s, self.latest_blocks_time)
            if self.has_received_blocks else []
        )
        enemy_only = (
            self._active_tracked_obstacles(self.tracked_enemy, now_time, self.enemy_stale_timeout_s, self.latest_enemy_time)
            if self.has_received_enemy else []
        )
        blocks = blocks_only + enemy_only

        now = now_time.to_msg()
        if not self.has_received_robot_pose:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.map_frame
            t.child_frame_id = 'camera_robot'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

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
        marker_id = self._append_zone_markers(
            marker_array,
            now,
            marker_id,
            ns='zones_interdites',
            names=self.zones_interdites_noms,
            xs=self.zones_interdites_centre_x_m,
            ys=self.zones_interdites_centre_y_m,
            size_xs=self.zones_interdites_taille_x_m,
            size_ys=self.zones_interdites_taille_y_m,
            color=(0.95, 0.2, 0.2, 0.28),
        )

        block_points: List[List[float]] = []
        static_points: List[List[float]] = []
        enemy_points: List[List[float]] = []
        self._append_zone_obstacle_points(
            static_points,
            xs=self.zones_interdites_centre_x_m,
            ys=self.zones_interdites_centre_y_m,
            size_xs=self.zones_interdites_taille_x_m,
            size_ys=self.zones_interdites_taille_y_m,
        )

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

            if str(block.get('color', 'unknown')).lower() == 'enemy_robot':
                self._append_detected_obstacle_points(enemy_points, block)
            else:
                self._append_detected_obstacle_points(block_points, block)

        self.marker_pub.publish(marker_array)

        if self.publish_block_obstacles:
            header = Header()
            header.stamp = now if self.stamp_obstacle_cloud_with_now else rclpy.time.Time().to_msg()
            header.frame_id = self.map_frame
            dynamic_cloud = point_cloud2.create_cloud_xyz32(header, block_points)
            self.block_pc_pub.publish(dynamic_cloud)
            static_obstacle_points = list(static_points)
            if self.publish_arena_walls_as_obstacles:
                static_obstacle_points.extend(self._wall_points)
            static_cloud = point_cloud2.create_cloud_xyz32(header, static_obstacle_points)
            self.static_pc_pub.publish(static_cloud)
            enemy_cloud = point_cloud2.create_cloud_xyz32(header, enemy_points)
            self.enemy_pc_pub.publish(enemy_cloud)
            self.dynamic_grid_pub.publish(self._build_dynamic_obstacle_grid(blocks_only, header))
            self.enemy_grid_pub.publish(self._build_dynamic_obstacle_grid(enemy_only, header))

    def _update_tracked_obstacles(self, detections: List[Dict[str, Any]], current_tracks: List[Dict[str, Any]], max_miss_count: int) -> List[Dict[str, Any]]:
        match_gate = max(self.dynamic_obstacle_match_distance_m, 0.0)
        matched_track_indices: set = set()

        for detection in detections:
            marker_id = int(detection.get('marker_id', -1))
            color = str(detection.get('color', 'unknown'))
            obs_x = float(detection.get('x', 0.0))
            obs_y = float(detection.get('y', 0.0))
            best_index = None
            best_distance = match_gate

            for i, track in enumerate(current_tracks):
                if i in matched_track_indices:
                    continue
                block = track['block']
                if int(block.get('marker_id', -2)) != marker_id:
                    continue
                if str(block.get('color', 'unknown')) != color:
                    continue
                dx = obs_x - float(block.get('x', 0.0))
                dy = obs_y - float(block.get('y', 0.0))
                distance = math.hypot(dx, dy)
                if distance <= best_distance:
                    best_distance = distance
                    best_index = i

            if best_index is None:
                current_tracks.append({'block': dict(detection), 'miss_count': 0})
                matched_track_indices.add(len(current_tracks) - 1)
            else:
                current_tracks[best_index] = {'block': dict(detection), 'miss_count': 0}
                matched_track_indices.add(best_index)

        max_miss = max(max_miss_count, 1)
        updated: List[Dict[str, Any]] = []
        for i, track in enumerate(current_tracks):
            if i not in matched_track_indices:
                track['miss_count'] = track.get('miss_count', 0) + 1
            if track.get('miss_count', 0) < max_miss:
                updated.append(track)
        return updated

    def _active_tracked_obstacles(self, tracks: List[Dict[str, Any]], now_time, stale_timeout_s: float, last_received_time=None) -> List[Dict[str, Any]]:
        ref_time = last_received_time if last_received_time is not None else self.latest_blocks_time
        age_s = (now_time - ref_time).nanoseconds * 1e-9
        if age_s > max(stale_timeout_s, 0.0):
            return []
        return [dict(track['block']) for track in tracks]

    def _build_dynamic_obstacle_grid(self, blocks: List[Dict[str, Any]], header: Header) -> OccupancyGrid:
        resolution = max(self.dynamic_obstacle_grid_resolution_m, 0.01)
        origin_x = self.arena_x_min - resolution
        origin_y = self.arena_y_min - resolution
        width = max(1, int(math.ceil((self.arena_x_max - self.arena_x_min) / resolution)) + 2)
        height = max(1, int(math.ceil((self.arena_y_max - self.arena_y_min) / resolution)) + 2)

        grid = OccupancyGrid()
        grid.header = header
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (width * height)

        for block in blocks[: self.max_blocks_visualized]:
            self._rasterize_dynamic_obstacle(grid, block)
        return grid

    def _fill_rect_cells(self, grid: OccupancyGrid, cx: float, cy: float,
                         cos_yaw: float, sin_yaw: float,
                         size_x: float, size_y: float, value: int) -> None:
        resolution = grid.info.resolution
        radius = 0.5 * math.hypot(size_x, size_y) + resolution
        min_ix = max(0, int(math.floor((cx - radius - grid.info.origin.position.x) / resolution)))
        max_ix = min(grid.info.width - 1, int(math.ceil((cx + radius - grid.info.origin.position.x) / resolution)))
        min_iy = max(0, int(math.floor((cy - radius - grid.info.origin.position.y) / resolution)))
        max_iy = min(grid.info.height - 1, int(math.ceil((cy + radius - grid.info.origin.position.y) / resolution)))
        half_x = 0.5 * size_x
        half_y = 0.5 * size_y
        for iy in range(min_iy, max_iy + 1):
            y = grid.info.origin.position.y + (iy + 0.5) * resolution
            for ix in range(min_ix, max_ix + 1):
                x = grid.info.origin.position.x + (ix + 0.5) * resolution
                dx = x - cx
                dy = y - cy
                local_x = dx * cos_yaw + dy * sin_yaw
                local_y = -dx * sin_yaw + dy * cos_yaw
                if abs(local_x) <= half_x + 0.5 * resolution and abs(local_y) <= half_y + 0.5 * resolution:
                    idx = iy * grid.info.width + ix
                    if grid.data[idx] < value:
                        grid.data[idx] = value

    def _rasterize_dynamic_obstacle(self, grid: OccupancyGrid, obstacle: Dict[str, Any]) -> None:
        center_x = float(obstacle.get('x', 0.0))
        center_y = float(obstacle.get('y', 0.0))
        color = str(obstacle.get('color', 'unknown')).lower()
        is_enemy = (color == 'enemy_robot')
        if is_enemy:
            padding = max(self.enemy_robot_obstacle_padding_m, 0.0)
        else:
            padding = max(self.detected_obstacle_padding_m, 0.0)
        size_x = max(float(obstacle.get('size_x_m', 0.12)) + 2.0 * padding, 0.03)
        size_y = max(float(obstacle.get('size_y_m', 0.05)) + 2.0 * padding, 0.03)
        yaw = float(obstacle.get('yaw', 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        if is_enemy:
            # Enemy robot: 3-ring gradient, all non-lethal (layer uses lethal_cost_threshold: 101).
            # Outer 30 cm ring (OGM 60 → ~150 cost), inner 10 cm ring (OGM 75 → ~188 cost),
            # core (OGM 100 → ~250 cost). Planner strongly avoids but can route through if needed;
            # collision monitor provides the real-time hard stop.
            self._fill_rect_cells(grid, center_x, center_y, cos_yaw, sin_yaw,
                                  size_x + 0.60, size_y + 0.60, 60)
            self._fill_rect_cells(grid, center_x, center_y, cos_yaw, sin_yaw,
                                  size_x + 0.20, size_y + 0.20, 75)
            self._fill_rect_cells(grid, center_x, center_y, cos_yaw, sin_yaw, size_x, size_y, 100)
        else:
            # Blocks: 2 cm shadow ring (OGM 30 → ~117 cost), core (OGM 60 → ~233 cost).
            # Layer uses lethal_cost_threshold: 65, so both values are non-lethal.
            self._fill_rect_cells(grid, center_x, center_y, cos_yaw, sin_yaw,
                                  size_x + 0.04, size_y + 0.04, 30)
            self._fill_rect_cells(grid, center_x, center_y, cos_yaw, sin_yaw, size_x, size_y, 60)

    def _build_wall_points(self) -> List[List[float]]:
        pts: List[List[float]] = []
        spacing = max(self.arena_wall_spacing_m, 0.01)
        z = self.arena_wall_z_m
        padding = max(self.static_obstacle_padding_m, 0.0)
        x_min, x_max = self.arena_x_min, self.arena_x_max
        y_min, y_max = self.arena_y_min, self.arena_y_max

        x_steps = max(1, int(math.ceil((x_max - x_min) / spacing)))
        pad_steps = max(1, int(math.ceil(max(padding, spacing) / spacing)))

        for ix in range(x_steps + 1):
            x = x_min + min(ix * spacing, x_max - x_min)
            for ip in range(pad_steps + 1):
                offset = min(ip * spacing, padding)
                pts.append([x, y_min + offset, z])
                pts.append([x, y_max - offset, z])

        y_steps = max(1, int(math.ceil((y_max - y_min) / spacing)))
        for iy in range(y_steps + 1):
            y = y_min + min(iy * spacing, y_max - y_min)
            for ip in range(pad_steps + 1):
                offset = min(ip * spacing, padding)
                pts.append([x_min + offset, y, z])
                pts.append([x_max - offset, y, z])

        return pts

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

    def _append_zone_obstacle_points(
        self,
        obstacle_points: List[List[float]],
        xs: List[float],
        ys: List[float],
        size_xs: List[float],
        size_ys: List[float],
    ) -> None:
        count = min(len(xs), len(ys), len(size_xs), len(size_ys))
        spacing = max(self.zones_interdites_point_spacing_m, 0.02)
        point_z = max(self.block_obstacle_height_m * 0.5, 0.03)

        padding = max(self.static_obstacle_padding_m, 0.0)

        for i in range(count):
            size_x = max(float(size_xs[i]) + 2.0 * padding, spacing)
            size_y = max(float(size_ys[i]) + 2.0 * padding, spacing)
            x_min = float(xs[i]) - (size_x * 0.5)
            y_min = float(ys[i]) - (size_y * 0.5)

            steps_x = max(1, int(math.ceil(size_x / spacing)))
            steps_y = max(1, int(math.ceil(size_y / spacing)))

            for ix in range(steps_x + 1):
                x = x_min + min(ix * spacing, size_x)
                for iy in range(steps_y + 1):
                    y = y_min + min(iy * spacing, size_y)
                    obstacle_points.append([x, y, point_z])

    def _append_detected_obstacle_points(
        self,
        obstacle_points: List[List[float]],
        obstacle: Dict[str, Any],
    ) -> None:
        center_x = float(obstacle.get('x', 0.0))
        center_y = float(obstacle.get('y', 0.0))
        center_z = float(obstacle.get('z', max(self.block_obstacle_height_m * 0.5, 0.03)))
        color = str(obstacle.get('color', 'unknown')).lower()
        if color == 'enemy_robot':
            padding = max(self.enemy_robot_obstacle_padding_m, 0.0)
        else:
            padding = max(self.detected_obstacle_padding_m, 0.0)
        size_x = max(float(obstacle.get('size_x_m', 0.12)) + 2.0 * padding, 0.03)
        size_y = max(float(obstacle.get('size_y_m', 0.05)) + 2.0 * padding, 0.03)
        yaw = float(obstacle.get('yaw', 0.0))
        spacing = max(self.detected_obstacle_point_spacing_m, 0.02)

        steps_x = max(1, int(math.ceil(size_x / spacing)))
        steps_y = max(1, int(math.ceil(size_y / spacing)))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for ix in range(steps_x + 1):
            local_x = -0.5 * size_x + min(ix * spacing, size_x)
            for iy in range(steps_y + 1):
                local_y = -0.5 * size_y + min(iy * spacing, size_y)
                x = center_x + local_x * cos_yaw - local_y * sin_yaw
                y = center_y + local_x * sin_yaw + local_y * cos_yaw
                obstacle_points.append([x, y, center_z])

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
        elif color_name == 'enemy_robot':
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
        elif color_name == 'black':
            marker.color.r = 0.02
            marker.color.g = 0.02
            marker.color.b = 0.02
        else:
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7

    def _declare_string_array_parameter(self, name: str) -> List[str]:
        value = self.declare_parameter(
            name,
            ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=[]),
        ).value
        return list(value)

    def _declare_double_array_parameter(self, name: str) -> List[float]:
        value = self.declare_parameter(
            name,
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[]),
        ).value
        return [float(v) for v in value]


def main(args=None) -> None:
    node = None
    try:
        _startup_log('main entered')
        rclpy.init(args=args)
        _startup_log('rclpy.init complete')
        node = CameraMapVisualizer()
        _startup_log('CameraMapVisualizer constructed')
        rclpy.spin(node)
        _startup_log('rclpy.spin returned')
    except (KeyboardInterrupt, ExternalShutdownException):
        _startup_log('shutdown requested')
    except Exception:
        _startup_log(traceback.format_exc())
        traceback.print_exc(file=sys.stderr)
        if node is not None:
            node.get_logger().fatal(
                'camera_map_visualizer_node crashed; see traceback above'
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
