import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


MATCH_DURATION = 98.0  # seconds
OPENER_STOCK_CLEARANCE_M = 0.10
# Front/back extent from the Nav2 full-navigation footprint.
OPENER_ROBOT_FRONT_EXTENT_M = 0.200
OPENER_ROBOT_REAR_EXTENT_M = 0.200
OPENER_ROBOT_HALF_WIDTH_M = 0.150
OPENER_TABLE_WIDTH_M = 3.000


class MatchNode(Node):

    def __init__(self):
        super().__init__('match_node')

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._running_pub = self.create_publisher(Bool, 'match/running', state_qos)
        self._opener_active_pub = self.create_publisher(Bool, 'match/opener_active', state_qos)
        self._closer_active_pub = self.create_publisher(Bool, 'match/closer_active', state_qos)

        self.declare_parameter('autostart', False)
        self.declare_parameter('duration_s', MATCH_DURATION)
        self.declare_parameter('force_running', False)
        self.declare_parameter('team_color', 'blue')
        self.declare_parameter('enable_opener', False)
        self.declare_parameter('enable_closer', False)
        self.declare_parameter('opener_nav_action_name', 'navigate_to_pose')
        self.declare_parameter('closer_planner_action_name', 'compute_path_to_pose')
        self.declare_parameter('opener_dock_action_name', 'dock_to_block')
        self.declare_parameter('opener_pick_action_name', 'pick')
        self.declare_parameter('opener_dispense_action_name', 'dispense')
        self.declare_parameter('opener_therm_action_name', 'therm')
        self.declare_parameter('opener_dock_timeout_s', 30.0)
        self.declare_parameter('opener_pick_timeout_s', 0.0)
        self.declare_parameter('opener_dispense_timeout_s', 0.0)
        self.declare_parameter('opener_therm_timeout_s', 0.0)
        self.declare_parameter('opener_dispense_count', 1)
        self.declare_parameter('opener_therm_direction', 'auto')
        self.declare_parameter('opener_step_retries', 3)
        self.declare_parameter('closer_start_before_end_s', 40.0)
        self.declare_parameter('closer_final_before_end_s', 15.0)
        self.declare_parameter('closer_plan_timeout_s', 5.0)
        self.declare_parameter('closer_cluster_topic', '/cluster_info')
        self.declare_parameter('closer_pickups_before_nid', 2)

        self._duration = Duration(seconds=float(self.get_parameter('duration_s').value))
        self._force_running = bool(self.get_parameter('force_running').value)
        self._team_color = self._normalize_team_color(self.get_parameter('team_color').value)
        self._enable_opener = bool(self.get_parameter('enable_opener').value)
        self._enable_closer = bool(self.get_parameter('enable_closer').value)
        self._opener_dock_timeout_s = float(self.get_parameter('opener_dock_timeout_s').value)
        self._opener_pick_timeout_s = float(self.get_parameter('opener_pick_timeout_s').value)
        self._opener_dispense_timeout_s = float(self.get_parameter('opener_dispense_timeout_s').value)
        self._opener_therm_timeout_s = float(self.get_parameter('opener_therm_timeout_s').value)
        self._opener_dispense_count = int(self.get_parameter('opener_dispense_count').value)
        self._opener_therm_direction = self.get_parameter('opener_therm_direction').value
        self._opener_step_retries = int(self.get_parameter('opener_step_retries').value)
        self._closer_start_before_end = Duration(
            seconds=float(self.get_parameter('closer_start_before_end_s').value)
        )
        self._closer_final_before_end = Duration(
            seconds=float(self.get_parameter('closer_final_before_end_s').value)
        )
        self._closer_plan_timeout_s = float(self.get_parameter('closer_plan_timeout_s').value)
        self._closer_pickups_before_nid = int(self.get_parameter('closer_pickups_before_nid').value)
        self._start_time = None
        self._running = False
        self._closer_started = False
        self._closer_finished = False
        self._closer_goal_candidates = []
        self._closer_candidate_index = 0
        self._closer_plan_timeout_timer = None
        self._closer_active_plan_handle = None
        self._closer_active_nav_handle = None
        self._closer_active_pick_handle = None
        self._closer_active_dispense_handle = None
        self._closer_plan_attempt_id = 0
        self._closer_nav_attempt_id = 0
        self._closer_current_zone_name = None
        self._closer_zone_deadline_time = 0.0
        self._closer_final_started = False
        self._closer_successful_pickups = 0
        self._closer_blocks_picked_count = 0
        self._closer_last_pick_count = 0
        self._closer_nav_goal_kind = 'garde_manger'
        self._last_cluster_info = None
        self._opener_started = False
        self._opener_finished = False
        self._opener_stage_index = 0
        self._opener_stages = self._build_opener_stages()
        self._opener_nav_wait_timer = None
        self._opener_retry_timer = None
        self._opener_retry_counts = {}
        self._last_best_pickup = None
        self._dock_action_type = None
        self._pick_action_type = None
        self._dispense_action_type = None
        self._therm_action_type = None

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('opener_nav_action_name').value,
        )
        self._planner_client = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter('closer_planner_action_name').value,
        )
        self._dock_client = None
        self._pick_client = None
        self._dispense_client = None
        self._therm_client = None

        self._tick = self.create_timer(0.1, self._on_tick)

        self.create_subscription(Bool, 'match/start', self._on_start, 10)
        self.create_subscription(String, '/best_pickup', self._on_best_pickup, 10)
        self.create_subscription(
            String,
            self.get_parameter('closer_cluster_topic').value,
            self._on_cluster_info,
            10,
        )

        if self._force_running:
            self._start_time = self.get_clock().now()
            self._running = True
            self._publish_state()
            self.get_logger().info('Match running forced true for testing.')
            self._start_opener_if_needed()
        elif bool(self.get_parameter('autostart').value):
            self._begin_match()
        else:
            self._publish_state()
            self.get_logger().info('Match node ready. Publish True to match/start to begin.')

    def _on_start(self, msg: Bool):
        if self._force_running:
            return

        if msg.data:
            if not self._running:
                self._begin_match()
        elif self._running:
            self._end_match('Match stopped.')

    def _on_tick(self):
        if self._force_running:
            if not self._running:
                self._running = True
            self._publish_state()
            self._start_opener_if_needed()
            self._start_closer_final_if_needed()
            self._start_closer_if_needed()
            return

        if not self._running or self._start_time is None:
            self._publish_activity_state()
            return

        self._publish_activity_state()
        self._start_closer_final_if_needed()
        self._start_closer_if_needed()

        if self.get_clock().now() - self._start_time >= self._duration:
            self._end_match(
                f'Match ended after {self._duration.nanoseconds / 1e9:.1f} s of ROS time.'
            )

    def _begin_match(self):
        self._start_time = self.get_clock().now()
        self._running = True
        self._publish_state()
        self.get_logger().info(
            f'Match began at ROS time {self._start_time.nanoseconds / 1e9:.3f} s.'
        )
        self._start_opener_if_needed()

    def _end_match(self, log_message: str):
        self._running = False
        self._opener_finished = True
        self._closer_finished = True
        self._publish_state()
        self.get_logger().info(log_message)

    def _publish_state(self):
        running_msg = Bool()
        running_msg.data = self._running
        self._running_pub.publish(running_msg)
        self._publish_activity_state()

    def _publish_activity_state(self):
        opener_msg = Bool()
        opener_msg.data = self._opener_started and not self._opener_finished
        self._opener_active_pub.publish(opener_msg)

        closer_msg = Bool()
        closer_msg.data = self._closer_started and not self._closer_finished
        self._closer_active_pub.publish(closer_msg)

    def _on_best_pickup(self, msg: String):
        try:
            self._last_best_pickup = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Could not parse /best_pickup JSON for opener pick goal')

    def _on_cluster_info(self, msg: String):
        try:
            self._last_cluster_info = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Could not parse /cluster_info JSON for closer')

    def _start_closer_if_needed(self):
        if (
            not self._enable_closer
            or not self._running
            or self._closer_started
            or self._closer_finished
            or self._closer_final_started
            or self._start_time is None
        ):
            return

        elapsed = self.get_clock().now() - self._start_time
        closer_start_time = self._duration - self._closer_start_before_end
        if elapsed >= closer_start_time:
            self.closer()

    def _start_closer_final_if_needed(self):
        if (
            not self._enable_closer
            or not self._running
            or self._closer_final_started
            or self._start_time is None
        ):
            return

        elapsed = self.get_clock().now() - self._start_time
        final_start_time = self._duration - self._closer_final_before_end
        if elapsed >= final_start_time:
            self._start_closer_final_actions()

    def _start_closer_final_actions(self):
        self._closer_final_started = True
        self._closer_started = True
        self._closer_finished = False
        self._opener_finished = True
        self._cancel_closer_plan_timeout()
        self._cancel_closer_active_goal_handles()
        self._publish_activity_state()

        if not self._ensure_manip_action_types():
            self._finish_closer('Closer final override stopped because action types are unavailable')
            return

        self.get_logger().info('Closer final override started: returning to nid for final drop')
        self._send_closer_nid_goal()

    def closer(self):
        self._closer_started = True
        self._publish_activity_state()
        if not self._ensure_manip_action_types():
            self._closer_finished = True
            self._publish_activity_state()
            return

        self._closer_goal_candidates = self._build_closer_goal_candidates()
        self._closer_candidate_index = 0
        self._closer_current_zone_name = None
        self._closer_zone_deadline_time = 0.0
        self._closer_final_started = False
        self._closer_successful_pickups = 0
        self._closer_blocks_picked_count = 0
        self._closer_last_pick_count = 0
        self._closer_nav_goal_kind = 'garde_manger'

        if not self._closer_goal_candidates:
            self._closer_finished = True
            self._publish_activity_state()
            self.get_logger().warning('Closer found no garde-manger containing enemy blocks')
            return

        best_zone = self._closer_goal_candidates[0]['zone_name']
        self.get_logger().info(
            f'Closer started with {len(self._closer_goal_candidates)} candidate poses; '
            f'best zone is {best_zone}'
        )
        self._try_next_closer_plan()

    def _build_closer_goal_candidates(self):
        if self._last_cluster_info is None:
            return []

        summaries = self._last_cluster_info.get('garde_manger_summary', [])
        zones_with_enemy_blocks = [
            zone for zone in summaries
            if int(zone.get('enemy_count', 0)) > 0
        ]
        zones_with_enemy_blocks.sort(
            key=lambda zone: (
                int(zone.get('enemy_count', 0)),
                int(zone.get('total_count', 0)),
            ),
            reverse=True,
        )

        robot_xy = self._closer_robot_xy()
        candidates = []
        for zone in zones_with_enemy_blocks:
            poses = self._closer_zone_goal_poses(zone)
            if robot_xy is not None:
                poses.sort(
                    key=lambda candidate: math.hypot(
                        candidate['x'] - robot_xy[0],
                        candidate['y'] - robot_xy[1],
                    )
                )
            candidates.extend(poses)
        return candidates

    def _closer_robot_xy(self):
        if self._last_cluster_info is None:
            return None
        robot_pose = (
            self._last_cluster_info
            .get('step_2_selection', {})
            .get('robot_pose_camera_xy')
        )
        if not isinstance(robot_pose, list) or len(robot_pose) < 2:
            return None
        return float(robot_pose[0]), float(robot_pose[1])

    def _closer_zone_goal_poses(self, zone):
        bounds = zone.get('bounds', {})
        center = zone.get('center', [])
        if len(center) < 2:
            return []

        x_min = float(bounds.get('x_min', center[0]))
        x_max = float(bounds.get('x_max', center[0]))
        y_min = float(bounds.get('y_min', center[1]))
        y_max = float(bounds.get('y_max', center[1]))
        center_x = float(center[0])
        center_y = float(center[1])
        stand_off = OPENER_STOCK_CLEARANCE_M + OPENER_ROBOT_FRONT_EXTENT_M

        raw_candidates = [
            {
                'zone_name': zone.get('name', 'garde_manger'),
                'x': x_min - stand_off,
                'y': center_y,
                'yaw': 0.0,
                'side': 'left',
                'enemy_count': int(zone.get('enemy_count', 0)),
            },
            {
                'zone_name': zone.get('name', 'garde_manger'),
                'x': x_max + stand_off,
                'y': center_y,
                'yaw': math.pi,
                'side': 'right',
                'enemy_count': int(zone.get('enemy_count', 0)),
            },
            {
                'zone_name': zone.get('name', 'garde_manger'),
                'x': center_x,
                'y': y_min - stand_off,
                'yaw': math.pi / 2.0,
                'side': 'bottom',
                'enemy_count': int(zone.get('enemy_count', 0)),
            },
            {
                'zone_name': zone.get('name', 'garde_manger'),
                'x': center_x,
                'y': y_max + stand_off,
                'yaw': -math.pi / 2.0,
                'side': 'top',
                'enemy_count': int(zone.get('enemy_count', 0)),
            },
        ]
        return [
            candidate for candidate in raw_candidates
            if self._closer_pose_fits_table(candidate['x'], candidate['y'], candidate['yaw'])
        ]

    def _closer_pose_fits_table(self, x: float, y: float, yaw: float) -> bool:
        facing_x = abs(math.cos(yaw)) > abs(math.sin(yaw))
        if facing_x:
            x_min = x - OPENER_ROBOT_REAR_EXTENT_M
            x_max = x + OPENER_ROBOT_FRONT_EXTENT_M
            y_min = y - OPENER_ROBOT_HALF_WIDTH_M
            y_max = y + OPENER_ROBOT_HALF_WIDTH_M
        else:
            x_min = x - OPENER_ROBOT_HALF_WIDTH_M
            x_max = x + OPENER_ROBOT_HALF_WIDTH_M
            y_min = y - OPENER_ROBOT_REAR_EXTENT_M
            y_max = y + OPENER_ROBOT_FRONT_EXTENT_M

        return (
            x_min >= 0.0
            and x_max <= OPENER_TABLE_WIDTH_M
            and y_min >= 0.0
            and y_max <= 2.0
        )

    def _try_next_closer_plan(self):
        if self._closer_finished:
            return

        if self._closer_candidate_index >= len(self._closer_goal_candidates):
            self._closer_finished = True
            self._publish_activity_state()
            self.get_logger().error('Closer could not find a safely plannable garde-manger target')
            return

        candidate = self._closer_goal_candidates[self._closer_candidate_index]
        now = self.get_clock().now().nanoseconds / 1e9
        if candidate['zone_name'] != self._closer_current_zone_name:
            self._closer_current_zone_name = candidate['zone_name']
            self._closer_zone_deadline_time = now + self._closer_plan_timeout_s

        remaining_plan_time_s = self._closer_zone_deadline_time - now
        if remaining_plan_time_s <= 0.0:
            self.get_logger().warning(
                f'Closer could not confirm a safe path to {candidate["zone_name"]} '
                f'within {self._closer_plan_timeout_s:.1f}s; trying another zone'
            )
            self._skip_current_closer_zone()
            return

        if not self._planner_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning('compute_path_to_pose action server not available for closer')
            self._skip_current_closer_zone()
            return

        self._closer_plan_attempt_id += 1
        attempt_id = self._closer_plan_attempt_id

        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_from_xy_yaw(candidate['x'], candidate['y'], candidate['yaw'])
        goal.use_start = False
        goal.planner_id = ''

        self.get_logger().info(
            f'Closer planning to {candidate["zone_name"]} from {candidate["side"]} side '
            f'({candidate["x"]:.3f}, {candidate["y"]:.3f})'
        )

        send_future = self._planner_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_closer_plan_goal_response(future, attempt_id)
        )
        self._start_closer_plan_timeout(attempt_id, remaining_plan_time_s)

    def _start_closer_plan_timeout(self, attempt_id: int, timeout_s: float):
        if self._closer_plan_timeout_timer is not None:
            self._closer_plan_timeout_timer.cancel()

        def on_timeout():
            if self._closer_plan_timeout_timer is not None:
                self._closer_plan_timeout_timer.cancel()
                self._closer_plan_timeout_timer = None
            if attempt_id != self._closer_plan_attempt_id or self._closer_finished:
                return
            candidate = self._closer_goal_candidates[self._closer_candidate_index]
            self.get_logger().warning(
                f'Closer could not confirm a safe path to {candidate["zone_name"]} '
                f'within {self._closer_plan_timeout_s:.1f}s; trying another zone'
            )
            if self._closer_active_plan_handle is not None:
                self._closer_active_plan_handle.cancel_goal_async()
                self._closer_active_plan_handle = None
            self._skip_current_closer_zone()

        self._closer_plan_timeout_timer = self.create_timer(
            max(0.1, timeout_s),
            on_timeout,
        )

    def _on_closer_plan_goal_response(self, future, attempt_id: int):
        if attempt_id != self._closer_plan_attempt_id or self._closer_finished:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._cancel_closer_plan_timeout()
            self.get_logger().warning(f'Closer plan request failed: {exc}')
            self._advance_closer_candidate()
            return

        if not goal_handle.accepted:
            self._cancel_closer_plan_timeout()
            self.get_logger().warning('Closer plan goal was rejected')
            self._advance_closer_candidate()
            return

        self._closer_active_plan_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_closer_plan_result(result, attempt_id)
        )

    def _on_closer_plan_result(self, future, attempt_id: int):
        if attempt_id != self._closer_plan_attempt_id or self._closer_finished:
            return

        self._cancel_closer_plan_timeout()
        self._closer_active_plan_handle = None

        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Closer plan result failed: {exc}')
            self._advance_closer_candidate()
            return

        path = getattr(result, 'path', None)
        path_len = 0 if path is None else len(path.poses)
        if getattr(result, 'error_code', 1) != 0 or path_len == 0:
            self.get_logger().warning(
                f'Closer planner found no valid path, error_code={getattr(result, "error_code", "unknown")}'
            )
            self._advance_closer_candidate()
            return

        candidate = self._closer_goal_candidates[self._closer_candidate_index]
        self.get_logger().info(
            f'Closer selected {candidate["zone_name"]}; path has {path_len} poses'
        )
        self._send_closer_nav_goal(candidate)

    def _cancel_closer_plan_timeout(self):
        if self._closer_plan_timeout_timer is not None:
            self._closer_plan_timeout_timer.cancel()
            self._closer_plan_timeout_timer = None

    def _advance_closer_candidate(self):
        self._closer_candidate_index += 1
        self._try_next_closer_plan()

    def _skip_current_closer_zone(self):
        if self._closer_candidate_index < len(self._closer_goal_candidates):
            failed_zone = self._closer_goal_candidates[self._closer_candidate_index]['zone_name']
            while (
                self._closer_candidate_index < len(self._closer_goal_candidates)
                and self._closer_goal_candidates[self._closer_candidate_index]['zone_name'] == failed_zone
            ):
                self._closer_candidate_index += 1
        self._try_next_closer_plan()

    def _send_closer_nav_goal(self, candidate):
        if not self._nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning('navigate_to_pose action server not available for closer')
            self._advance_closer_candidate()
            return

        self._closer_nav_goal_kind = 'garde_manger'
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._pose_from_xy_yaw(candidate['x'], candidate['y'], candidate['yaw'])
        self._closer_nav_attempt_id += 1
        attempt_id = self._closer_nav_attempt_id
        send_future = self._nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda future: self._on_closer_nav_goal_response(future, attempt_id, 'garde_manger')
        )

    def _on_closer_nav_goal_response(self, future, attempt_id: int, goal_kind: str):
        if attempt_id != self._closer_nav_attempt_id or self._closer_finished:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Closer Nav2 goal request failed: {exc}')
            self._handle_closer_nav_failure(goal_kind)
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Closer Nav2 goal was rejected')
            self._handle_closer_nav_failure(goal_kind)
            return

        self._closer_active_nav_handle = goal_handle
        if goal_kind == 'nid':
            self.get_logger().info('Closer Nav2 goal accepted for team nid')
        else:
            candidate = self._closer_goal_candidates[self._closer_candidate_index]
            self.get_logger().info(f'Closer Nav2 goal accepted for {candidate["zone_name"]}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_closer_nav_result(result, attempt_id, goal_kind)
        )

    def _on_closer_nav_result(self, future, attempt_id: int, goal_kind: str):
        if attempt_id != self._closer_nav_attempt_id or self._closer_finished:
            return

        self._closer_active_nav_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Closer Nav2 result failed: {exc}')
            self._handle_closer_nav_failure(goal_kind)
            return

        if not result or getattr(result, 'error_code', 0) != 0:
            self.get_logger().warning(
                f'Closer Nav2 goal failed with error_code={getattr(result, "error_code", "unknown")}'
            )
            self._handle_closer_nav_failure(goal_kind)
            return

        if goal_kind == 'nid':
            self.get_logger().info('Closer reached team nid; starting drop action')
            self._send_closer_dispense_goal()
            return

        candidate = self._closer_goal_candidates[self._closer_candidate_index]
        self.get_logger().info(f'Closer reached {candidate["zone_name"]}; starting pickup action')
        self._send_closer_pick_goal()

    def _handle_closer_nav_failure(self, goal_kind: str):
        if goal_kind == 'nid':
            self._finish_closer('Closer stopped because Nav2 could not reach the team nid')
            return
        self._advance_closer_candidate()

    def _send_closer_pick_goal(self):
        if not self._pick_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('pick action server is not available for closer')
            self._finish_closer('Closer stopped because pick action server is unavailable')
            return

        colors, count = self._build_pick_colors()
        if count == 0:
            self.get_logger().warning('No /best_pickup available for closer; using single-block fallback')
            colors = [1, 0, 0, 0]
            count = 1

        pick_goal = self._pick_action_type.Goal()
        pick_goal.colors = colors
        pick_goal.count = count
        pick_goal.timeout_sec = self._opener_pick_timeout_s
        self._closer_last_pick_count = count
        self.get_logger().info(f'Closer sending Pick goal: colors={colors}, count={count}')
        send_future = self._pick_client.send_goal_async(pick_goal)
        send_future.add_done_callback(self._on_closer_pick_goal_response)

    def _on_closer_pick_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Closer pick goal request failed: {exc}')
            self._finish_closer('Closer stopped after pick goal request failure')
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Closer pick goal was rejected')
            self._finish_closer('Closer stopped after pick goal rejection')
            return

        if self._closer_final_started:
            goal_handle.cancel_goal_async()
            return

        self._closer_active_pick_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_closer_pick_result)

    def _on_closer_pick_result(self, future):
        if self._closer_final_started:
            self._closer_active_pick_handle = None
            return

        self._closer_active_pick_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Closer pick result failed: {exc}')
            self._finish_closer('Closer stopped after pick result failure')
            return

        if not getattr(result, 'success', False):
            self.get_logger().warning(f'Closer pickup failed: {getattr(result, "message", "")}')
            self._finish_closer('Closer stopped after pickup failure')
            return

        picked_count = int(getattr(result, 'count', 0))
        if picked_count <= 0:
            picked_count = self._closer_last_pick_count
        self._closer_successful_pickups += 1
        self._closer_blocks_picked_count += max(0, picked_count)
        self.get_logger().info(f'Closer pickup succeeded: {getattr(result, "message", "")}')
        if self._closer_successful_pickups >= self._closer_pickups_before_nid:
            self._send_closer_nid_goal()
            return

        self._restart_closer_selection()

    def _send_closer_nid_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=0.2):
            self._finish_closer('Closer stopped because Nav2 is unavailable for team nid return')
            return

        self._closer_nav_goal_kind = 'nid'
        x, y, yaw = self._closer_nid_pose()
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._pose_from_xy_yaw(x, y, yaw)
        self.get_logger().info(f'Closer returning to team nid at ({x:.3f}, {y:.3f})')
        self._closer_nav_attempt_id += 1
        attempt_id = self._closer_nav_attempt_id
        send_future = self._nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda future: self._on_closer_nav_goal_response(future, attempt_id, 'nid')
        )

    def _closer_nid_pose(self):
        if self._team_color == 'yellow':
            return 0.300, 1.775, 0.0
        return 2.700, 1.775, math.pi

    def _send_closer_dispense_goal(self):
        if not self._dispense_client.wait_for_server(timeout_sec=2.0):
            self._finish_closer('Closer stopped because dispense action server is unavailable')
            return

        dispense_goal = self._dispense_action_type.Goal()
        dispense_goal.count = max(1, self._closer_blocks_picked_count)
        dispense_goal.timeout_sec = self._opener_dispense_timeout_s
        self.get_logger().info(f'Closer sending Dispense goal: count={dispense_goal.count}')
        send_future = self._dispense_client.send_goal_async(dispense_goal)
        send_future.add_done_callback(self._on_closer_dispense_goal_response)

    def _on_closer_dispense_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Closer dispense goal request failed: {exc}')
            self._finish_closer('Closer stopped after dispense goal request failure')
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Closer dispense goal was rejected')
            self._finish_closer('Closer stopped after dispense goal rejection')
            return

        self._closer_active_dispense_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_closer_dispense_result)

    def _on_closer_dispense_result(self, future):
        self._closer_active_dispense_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Closer dispense result failed: {exc}')
            self._finish_closer('Closer stopped after dispense result failure')
            return

        if not getattr(result, 'success', False):
            self.get_logger().warning(f'Closer dispense failed: {getattr(result, "message", "")}')
            self._finish_closer('Closer stopped after dispense failure')
            return

        self._finish_closer(f'Closer complete: dropped blocks in team nid ({getattr(result, "message", "")})')

    def _cancel_closer_active_goal_handles(self):
        for goal_handle in (
            self._closer_active_plan_handle,
            self._closer_active_nav_handle,
            self._closer_active_pick_handle,
            self._closer_active_dispense_handle,
        ):
            if goal_handle is None:
                continue
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Could not cancel closer goal during final override: {exc}')

        self._closer_active_plan_handle = None
        self._closer_active_nav_handle = None
        self._closer_active_pick_handle = None
        self._closer_active_dispense_handle = None

    def _restart_closer_selection(self):
        if not self._running:
            self._finish_closer('Closer stopped because match is no longer running')
            return

        self._closer_goal_candidates = self._build_closer_goal_candidates()
        self._closer_candidate_index = 0
        self._closer_current_zone_name = None
        self._closer_zone_deadline_time = 0.0

        if not self._closer_goal_candidates:
            self._finish_closer('Closer complete: no garde-manger currently contains enemy blocks')
            return

        best_zone = self._closer_goal_candidates[0]['zone_name']
        self.get_logger().info(f'Closer reselecting target; best zone is now {best_zone}')
        self._try_next_closer_plan()

    def _finish_closer(self, log_message: str):
        self._closer_finished = True
        self._cancel_closer_plan_timeout()
        self._publish_activity_state()
        self.get_logger().info(log_message)

    def _start_opener_if_needed(self):
        if not self._enable_opener or not self._running or self._opener_started:
            return

        self._opener_started = True
        self._publish_activity_state()
        if not self._ensure_manip_action_types():
            self._opener_finished = True
            self._publish_activity_state()
            return

        self.get_logger().info(
            f'Opener started for team={self._team_color}; '
            f'{len(self._opener_stages)} stages queued'
        )
        self._start_current_opener_stage()

    def _start_current_opener_stage(self):
        stage = self._current_opener_stage()
        if stage is None:
            self._opener_finished = True
            self._publish_activity_state()
            self.get_logger().info('Opener complete: all stages succeeded')
            return

        if stage['kind'] == 'therm':
            self._send_therm_goal()
            return

        self._try_send_opener_nav_goal()

    def _try_send_opener_nav_goal(self):
        if self._opener_finished:
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            if self._opener_nav_wait_timer is None:
                self.get_logger().info('Nav2 navigate_to_pose not available yet; opener is waiting.')
                self._opener_nav_wait_timer = self.create_timer(0.5, self._try_send_opener_nav_goal)
            return

        if self._opener_nav_wait_timer is not None:
            self._opener_nav_wait_timer.cancel()
            self._opener_nav_wait_timer = None

        stage = self._current_opener_stage()
        if stage is None:
            self._opener_finished = True
            self._publish_activity_state()
            self.get_logger().info('Opener complete: no more stages queued')
            return
        if stage['kind'] == 'therm':
            self._send_therm_goal()
            return

        goal_pose = self._opener_goal_pose(stage)
        self.get_logger().info(
            f'Opener navigating to {stage["name"]} ({goal_pose.pose.position.x:.3f}, '
            f'{goal_pose.pose.position.y:.3f})'
        )

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        send_future = self._nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(self._on_nav_goal_response)

    def _retry_or_abort_opener_step(self, step_name: str, retry_callback, failure_message: str):
        retry_key = (self._opener_stage_index, step_name)
        retries_used = self._opener_retry_counts.get(retry_key, 0)
        stage = self._current_opener_stage()
        stage_name = stage['name'] if stage else 'current stock'

        if retries_used < self._opener_step_retries:
            retries_used += 1
            self._opener_retry_counts[retry_key] = retries_used
            self.get_logger().warning(
                f'{failure_message}; retrying {step_name} for {stage_name} '
                f'({retries_used}/{self._opener_step_retries})'
            )
            self._schedule_opener_retry(retry_callback)
            return

        self.get_logger().error(
            f'{failure_message}; opener aborting after {self._opener_step_retries} retries'
        )
        self._opener_finished = True
        self._publish_activity_state()

    def _schedule_opener_retry(self, retry_callback):
        if self._opener_retry_timer is not None:
            self._opener_retry_timer.cancel()

        def retry_once():
            if self._opener_retry_timer is not None:
                self._opener_retry_timer.cancel()
                self._opener_retry_timer = None
            if not self._opener_finished:
                retry_callback()

        self._opener_retry_timer = self.create_timer(0.5, retry_once)

    def _ensure_manip_action_types(self) -> bool:
        if (
            self._dock_action_type is not None
            and self._pick_action_type is not None
            and self._dispense_action_type is not None
            and self._therm_action_type is not None
        ):
            return True

        try:
            from custom_msgs.action import Dispense, DockToBlock, Pick, Therm
        except ImportError as exc:
            self.get_logger().error(
                'Opener needs custom_msgs DockToBlock/Pick/Dispense/Therm actions. '
                'Source robot_ws after server_ws before launching. '
                f'Import error: {exc}'
            )
            return False

        self._dock_action_type = DockToBlock
        self._pick_action_type = Pick
        self._dispense_action_type = Dispense
        self._therm_action_type = Therm
        self._dock_client = ActionClient(
            self,
            DockToBlock,
            self.get_parameter('opener_dock_action_name').value,
        )
        self._pick_client = ActionClient(
            self,
            Pick,
            self.get_parameter('opener_pick_action_name').value,
        )
        self._dispense_client = ActionClient(
            self,
            Dispense,
            self.get_parameter('opener_dispense_action_name').value,
        )
        self._therm_client = ActionClient(
            self,
            Therm,
            self.get_parameter('opener_therm_action_name').value,
        )
        return True

    def _current_opener_stage(self):
        if self._opener_stage_index >= len(self._opener_stages):
            return None
        return self._opener_stages[self._opener_stage_index]

    def _build_opener_stages(self):
        left_stock_goal_x = 0.175
        right_stock_goal_x = 2.825
        upper_stock_goal_y = 1.300 + OPENER_STOCK_CLEARANCE_M + OPENER_ROBOT_FRONT_EXTENT_M
        lower_stock_goal_y = 0.500 + OPENER_STOCK_CLEARANCE_M + OPENER_ROBOT_FRONT_EXTENT_M
        left_wall_garde_manger_x = OPENER_ROBOT_HALF_WIDTH_M
        right_wall_garde_manger_x = OPENER_TABLE_WIDTH_M - OPENER_ROBOT_HALF_WIDTH_M
        lower_garde_manger_back_y = 0.700
        garde_manger_8_wall_edge_x = 0.600
        garde_manger_10_wall_edge_x = 2.400
        lower_garde_manger_goal_y = lower_garde_manger_back_y - OPENER_ROBOT_REAR_EXTENT_M
        garde_manger_8_goal_x = garde_manger_8_wall_edge_x - OPENER_ROBOT_REAR_EXTENT_M
        garde_manger_10_goal_x = garde_manger_10_wall_edge_x + OPENER_ROBOT_REAR_EXTENT_M

        if self._team_color == 'yellow':
            return [
                # Front edge 10 cm above stock_1's y=1.300 edge, facing -y.
                {'kind': 'stock', 'name': 'stock_1', 'x': left_stock_goal_x, 'y': upper_stock_goal_y, 'yaw': -math.pi / 2.0},
                # Front edge 10 cm above stock_2's y=0.500 edge, facing -y.
                {'kind': 'stock', 'name': 'stock_2', 'x': left_stock_goal_x, 'y': lower_stock_goal_y, 'yaw': -math.pi / 2.0},
                # Rear edge lined up with garde_manger_3's lower y edge.
                {'kind': 'drop', 'name': 'garde_manger_3', 'x': left_wall_garde_manger_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
                {'kind': 'therm', 'name': 'therm'},
                # Back side lined up with garde_manger_8's x=0.600 edge, facing -x.
                {'kind': 'drop', 'name': 'garde_manger_8', 'x': garde_manger_8_goal_x, 'y': 0.100, 'yaw': math.pi},
            ]

        return [
            # Front edge 10 cm above stock_5's y=1.300 edge, facing -y.
            {'kind': 'stock', 'name': 'stock_5', 'x': right_stock_goal_x, 'y': upper_stock_goal_y, 'yaw': -math.pi / 2.0},
            # Front edge 10 cm above stock_6's y=0.500 edge, facing -y.
            {'kind': 'stock', 'name': 'stock_6', 'x': right_stock_goal_x, 'y': lower_stock_goal_y, 'yaw': -math.pi / 2.0},
            # Rear edge lined up with garde_manger_7's lower y edge.
            {'kind': 'drop', 'name': 'garde_manger_7', 'x': right_wall_garde_manger_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
            {'kind': 'therm', 'name': 'therm'},
            # Back side lined up with garde_manger_10's x=2.400 edge, facing +x.
            {'kind': 'drop', 'name': 'garde_manger_10', 'x': garde_manger_10_goal_x, 'y': 0.100, 'yaw': 0.0},
        ]

    def _opener_goal_pose(self, stage) -> PoseStamped:
        return self._pose_from_xy_yaw(stage['x'], stage['y'], stage['yaw'])

    def _pose_from_xy_yaw(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _on_nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'navigation',
                self._try_send_opener_nav_goal,
                f'Opener Nav2 goal request failed: {exc}',
            )
            return

        if not goal_handle.accepted:
            self._retry_or_abort_opener_step(
                'navigation',
                self._try_send_opener_nav_goal,
                'Opener Nav2 goal was rejected',
            )
            return

        self.get_logger().info('Opener Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'navigation',
                self._try_send_opener_nav_goal,
                f'Opener Nav2 result failed: {exc}',
            )
            return

        if not result or getattr(result, 'error_code', 0) != 0:
            error_code = getattr(result, 'error_code', 'unknown')
            self._retry_or_abort_opener_step(
                'navigation',
                self._try_send_opener_nav_goal,
                f'Opener Nav2 goal failed with error_code={error_code}',
            )
            return

        stage = self._current_opener_stage()
        stage_name = stage['name'] if stage else 'current stock'
        if stage and stage['kind'] == 'drop':
            self.get_logger().info(f'Opener Nav2 goal to {stage_name} succeeded; starting dispense action')
            self._send_dispense_goal()
            return

        self.get_logger().info(f'Opener Nav2 goal to {stage_name} succeeded; starting docking action')
        self._send_dock_goal()

    def _send_dock_goal(self):
        if not self._dock_client.wait_for_server(timeout_sec=2.0):
            self._retry_or_abort_opener_step(
                'docking',
                self._send_dock_goal,
                'dock_to_block action server is not available',
            )
            return

        dock_goal = self._dock_action_type.Goal()
        dock_goal.timeout_sec = self._opener_dock_timeout_s
        send_future = self._dock_client.send_goal_async(dock_goal)
        send_future.add_done_callback(self._on_dock_goal_response)

    def _on_dock_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'docking',
                self._send_dock_goal,
                f'Opener dock goal request failed: {exc}',
            )
            return

        if not goal_handle.accepted:
            self._retry_or_abort_opener_step(
                'docking',
                self._send_dock_goal,
                'Opener dock goal was rejected',
            )
            return

        self.get_logger().info('Opener dock goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dock_result)

    def _on_dock_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'docking',
                self._send_dock_goal,
                f'Opener dock result failed: {exc}',
            )
            return

        if not getattr(result, 'success', False):
            message = getattr(result, 'message', '')
            self._retry_or_abort_opener_step(
                'docking',
                self._send_dock_goal,
                f'Opener docking failed: {message}',
            )
            return

        self.get_logger().info('Opener docking succeeded; starting pickup action')
        self._send_pick_goal()

    def _send_pick_goal(self):
        if not self._pick_client.wait_for_server(timeout_sec=2.0):
            self._retry_or_abort_opener_step(
                'pickup',
                self._send_pick_goal,
                'pick action server is not available',
            )
            return

        colors, count = self._build_pick_colors()
        if count == 0:
            self.get_logger().warning('No /best_pickup available for opener; using single-block fallback')
            colors = [1, 0, 0, 0]
            count = 1

        pick_goal = self._pick_action_type.Goal()
        pick_goal.colors = colors
        pick_goal.count = count
        pick_goal.timeout_sec = self._opener_pick_timeout_s
        self.get_logger().info(f'Opener sending Pick goal: colors={colors}, count={count}')
        send_future = self._pick_client.send_goal_async(pick_goal)
        send_future.add_done_callback(self._on_pick_goal_response)

    def _on_pick_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'pickup',
                self._send_pick_goal,
                f'Opener pick goal request failed: {exc}',
            )
            return

        if not goal_handle.accepted:
            self._retry_or_abort_opener_step(
                'pickup',
                self._send_pick_goal,
                'Opener pick goal was rejected',
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_pick_result)

    def _on_pick_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'pickup',
                self._send_pick_goal,
                f'Opener pick result failed: {exc}',
            )
            return

        if not getattr(result, 'success', False):
            self._retry_or_abort_opener_step(
                'pickup',
                self._send_pick_goal,
                f'Opener pickup failed: {getattr(result, "message", "")}',
            )
            return

        stage = self._current_opener_stage()
        stage_name = stage['name'] if stage else 'current stock'
        self.get_logger().info(f'Opener pickup at {stage_name} succeeded: {getattr(result, "message", "")}')

        self._opener_stage_index += 1
        self._continue_to_next_opener_stage()

    def _send_dispense_goal(self):
        if not self._dispense_client.wait_for_server(timeout_sec=2.0):
            self._retry_or_abort_opener_step(
                'dispense',
                self._send_dispense_goal,
                'dispense action server is not available',
            )
            return

        dispense_goal = self._dispense_action_type.Goal()
        dispense_goal.count = max(1, self._opener_dispense_count)
        dispense_goal.timeout_sec = self._opener_dispense_timeout_s
        self.get_logger().info(f'Opener sending Dispense goal: count={dispense_goal.count}')
        send_future = self._dispense_client.send_goal_async(dispense_goal)
        send_future.add_done_callback(self._on_dispense_goal_response)

    def _on_dispense_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'dispense',
                self._send_dispense_goal,
                f'Opener dispense goal request failed: {exc}',
            )
            return

        if not goal_handle.accepted:
            self._retry_or_abort_opener_step(
                'dispense',
                self._send_dispense_goal,
                'Opener dispense goal was rejected',
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dispense_result)

    def _on_dispense_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'dispense',
                self._send_dispense_goal,
                f'Opener dispense result failed: {exc}',
            )
            return

        if not getattr(result, 'success', False):
            self._retry_or_abort_opener_step(
                'dispense',
                self._send_dispense_goal,
                f'Opener dispense failed: {getattr(result, "message", "")}',
            )
            return

        stage = self._current_opener_stage()
        stage_name = stage['name'] if stage else 'current drop'
        self.get_logger().info(f'Opener dispense at {stage_name} succeeded: {getattr(result, "message", "")}')
        self._opener_stage_index += 1
        self._continue_to_next_opener_stage()

    def _send_therm_goal(self):
        if not self._therm_client.wait_for_server(timeout_sec=2.0):
            self._retry_or_abort_opener_step(
                'therm',
                self._send_therm_goal,
                'therm action server is not available',
            )
            return

        therm_goal = self._therm_action_type.Goal()
        therm_goal.direction = self._opener_therm_direction_value()
        therm_goal.timeout_sec = self._opener_therm_timeout_s
        self.get_logger().info(f'Opener sending Therm goal: direction={therm_goal.direction}')
        send_future = self._therm_client.send_goal_async(therm_goal)
        send_future.add_done_callback(self._on_therm_goal_response)

    def _on_therm_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'therm',
                self._send_therm_goal,
                f'Opener therm goal request failed: {exc}',
            )
            return

        if not goal_handle.accepted:
            self._retry_or_abort_opener_step(
                'therm',
                self._send_therm_goal,
                'Opener therm goal was rejected',
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_therm_result)

    def _on_therm_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._retry_or_abort_opener_step(
                'therm',
                self._send_therm_goal,
                f'Opener therm result failed: {exc}',
            )
            return

        if not getattr(result, 'success', False):
            self._retry_or_abort_opener_step(
                'therm',
                self._send_therm_goal,
                f'Opener therm failed: {getattr(result, "message", "")}',
            )
            return

        self.get_logger().info(f'Opener therm succeeded: {getattr(result, "message", "")}')
        self._opener_stage_index += 1
        self._continue_to_next_opener_stage()

    def _continue_to_next_opener_stage(self):
        next_stage = self._current_opener_stage()
        if next_stage is None:
            self._opener_finished = True
            self._publish_activity_state()
            self.get_logger().info('Opener complete: all stages succeeded')
            return

        self.get_logger().info(f'Opener continuing to {next_stage["name"]}')
        self._start_current_opener_stage()

    def _opener_therm_direction_value(self) -> int:
        direction = str(self._opener_therm_direction).strip().lower()
        if direction in ('left', 'gauche', '-1'):
            return -1
        if direction in ('right', 'droite', '1'):
            return 1
        if self._team_color == 'yellow':
            return 1
        return -1

    def _build_pick_colors(self):
        colors = [0, 0, 0, 0]
        if self._last_best_pickup is None:
            return colors, 0

        cup_index = {'cup_0': 0, 'cup_1': 1, 'cup_2': 2, 'cup_3': 3}
        for assignment in self._last_best_pickup.get('assignments', []):
            idx = cup_index.get(assignment.get('cup', ''), -1)
            if idx < 0:
                continue
            colors[idx] = self._color_to_pick_mode(assignment.get('color', 'unknown'))
        count = sum(1 for color in colors if color != 0)
        return colors, count

    def _color_to_pick_mode(self, color: str) -> int:
        normalized = self._normalize_team_color(color)
        if normalized == 'unknown' or normalized == self._team_color:
            return 1
        return 2

    @staticmethod
    def _normalize_team_color(color: str) -> str:
        normalized = str(color).strip().lower()
        if normalized in ('blue', 'bleu'):
            return 'blue'
        if normalized in ('yellow', 'jaune'):
            return 'yellow'
        return 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = MatchNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
