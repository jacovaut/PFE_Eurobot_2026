#!/usr/bin/env python3

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class ClusterGoalBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('cluster_goal_bridge_node')

        self.cluster_topic = self.declare_parameter('cluster_topic', '/cluster_info').value
        self.action_name = self.declare_parameter('action_name', 'navigate_to_pose').value
        self.goal_frame = self.declare_parameter('goal_frame', 'map').value

        self.enabled = bool(self.declare_parameter('enabled', True).value)
        self.min_score = float(self.declare_parameter('min_score', 0.0).value)
        self.approach_offset_m = float(self.declare_parameter('approach_offset_m', 0.18).value)
        self.min_goal_separation_m = float(self.declare_parameter('min_goal_separation_m', 0.08).value)
        self.min_goal_update_period_s = float(self.declare_parameter('min_goal_update_period_s', 0.7).value)

        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._last_goal_yaw: Optional[float] = None
        self._last_goal_sent_time: float = 0.0
        self._active_goal = None

        self._action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.create_subscription(String, self.cluster_topic, self._cluster_cb, 20)

        self.get_logger().info('cluster_goal_bridge_node started')

    def _cluster_cb(self, msg: String) -> None:
        if not self.enabled:
            return

        best_cluster, robot_xy = self._parse_best_cluster(msg.data)
        if best_cluster is None:
            return

        score = float(best_cluster.get('score', 0.0))
        if score < self.min_score:
            return

        center = best_cluster.get('center', None)
        if not isinstance(center, list) or len(center) < 2:
            return

        center_x = float(center[0])
        center_y = float(center[1])

        goal_x, goal_y = self._compute_goal_xy(center_x, center_y, robot_xy)
        goal_yaw = self._compute_goal_yaw(goal_x, goal_y, center_x, center_y)

        if not self._should_send_goal(goal_x, goal_y, goal_yaw):
            return

        self._send_nav_goal(goal_x, goal_y, goal_yaw)

    def _parse_best_cluster(self, data: str):
        try:
            parsed = json.loads(data)
            step_2 = parsed.get('step_2_selection', {})
            best_cluster = step_2.get('best_cluster', None)

            robot_xy = None
            robot_pose = step_2.get('robot_pose_camera_xy', None)
            if isinstance(robot_pose, list) and len(robot_pose) >= 2:
                robot_xy = (float(robot_pose[0]), float(robot_pose[1]))

            if isinstance(best_cluster, dict):
                return best_cluster, robot_xy
        except Exception as exc:
            self.get_logger().warning(f'Failed to parse cluster_info JSON: {exc}')
        return None, None

    def _compute_goal_xy(self, center_x: float, center_y: float, robot_xy: Optional[Tuple[float, float]]) -> Tuple[float, float]:
        if robot_xy is None or self.approach_offset_m <= 0.0:
            return center_x, center_y

        dx = center_x - robot_xy[0]
        dy = center_y - robot_xy[1]
        dist = math.hypot(dx, dy)

        if dist <= self.approach_offset_m + 1e-6:
            return center_x, center_y

        ux = dx / dist
        uy = dy / dist
        return center_x - self.approach_offset_m * ux, center_y - self.approach_offset_m * uy

    @staticmethod
    def _compute_goal_yaw(goal_x: float, goal_y: float, look_at_x: float, look_at_y: float) -> float:
        return math.atan2(look_at_y - goal_y, look_at_x - goal_x)

    def _should_send_goal(self, x: float, y: float, yaw: float) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_goal_sent_time < self.min_goal_update_period_s:
            return False

        if self._last_goal_xy is None or self._last_goal_yaw is None:
            return True

        delta_xy = math.hypot(x - self._last_goal_xy[0], y - self._last_goal_xy[1])
        delta_yaw = abs(math.atan2(math.sin(yaw - self._last_goal_yaw), math.cos(yaw - self._last_goal_yaw)))

        return delta_xy > self.min_goal_separation_m or delta_yaw > 0.30

    def _send_nav_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('navigate_to_pose action server not available yet')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        half = yaw * 0.5
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

        self._last_goal_xy = (x, y)
        self._last_goal_yaw = yaw
        self._last_goal_sent_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f'Sent goal from best cluster: x={x:.3f} y={y:.3f} yaw={yaw:.2f} rad')

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Cluster goal rejected by Nav2')
            return

        self._active_goal = goal_handle
        self.get_logger().info('Cluster goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        _ = feedback_msg

    def _result_cb(self, future) -> None:
        try:
            result = future.result()
            status = int(result.status)
            self.get_logger().info(f'Cluster goal finished with status={status}')
        except Exception as exc:
            self.get_logger().warning(f'Cluster goal result failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClusterGoalBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
