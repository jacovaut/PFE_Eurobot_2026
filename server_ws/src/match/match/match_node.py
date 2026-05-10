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
        self._team_color_pub = self.create_publisher(String, 'team_color', state_qos)

        self.declare_parameter('autostart', False)
        self.declare_parameter('duration_s', MATCH_DURATION)
        self.declare_parameter('force_running', False)
        self.declare_parameter('start_delay_s', 0.0)
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
        self.declare_parameter('end_before_end_s', 15.0)
        self.declare_parameter('closer_plan_timeout_s', 5.0)
        self.declare_parameter('closer_cluster_topic', '/cluster_info')
        self.declare_parameter('closer_pickups_before_nid', 2)
        self.declare_parameter('enable_midgame', False)
        self.declare_parameter('enable_end', False)
        self.declare_parameter('midgame_cluster_min_score', 0.0)
        self.declare_parameter('midgame_cluster_approach_offset_m', 0.18)

        self._duration = Duration(seconds=float(self.get_parameter('duration_s').value))
        self._force_running = bool(self.get_parameter('force_running').value)
        self._team_color = self._normalize_team_color(self.get_parameter('team_color').value)
        self._enable_opener = bool(self.get_parameter('enable_opener').value)
        self._enable_closer = bool(self.get_parameter('enable_closer').value)
        self._enable_midgame = bool(self.get_parameter('enable_midgame').value)
        self._enable_end = bool(self.get_parameter('enable_end').value)
        self._midgame_cluster_min_score = float(self.get_parameter('midgame_cluster_min_score').value)
        self._midgame_cluster_approach_offset_m = float(
            self.get_parameter('midgame_cluster_approach_offset_m').value
        )
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
        self._end_before_end = Duration(
            seconds=float(self.get_parameter('end_before_end_s').value)
        )
        self._closer_plan_timeout_s = float(self.get_parameter('closer_plan_timeout_s').value)
        self._closer_pickups_before_nid = int(self.get_parameter('closer_pickups_before_nid').value)
        self._start_time = None
        self._running = False
        self._start_delay_done = True
        self._start_delay_timer = None
        self._closer_started = False
        self._closer_finished = False
        self._closer_goal_candidates = []
        self._closer_candidate_index = 0
        self._closer_plan_timeout_timer = None
        self._closer_active_plan_handle = None
        self._closer_active_nav_handle = None
        self._closer_active_pick_handle = None
        self._closer_plan_attempt_id = 0
        self._closer_nav_attempt_id = 0
        self._closer_current_zone_name = None
        self._closer_zone_deadline_time = 0.0
        self._closer_successful_pickups = 0
        self._closer_blocks_picked_count = 0
        self._closer_last_pick_count = 0
        self._closer_pending_after_midgame_flush = False
        self._end_started = False
        self._end_finished = False
        self._end_nav_attempt_id = 0
        self._end_active_nav_handle = None
        self._end_active_dispense_handle = None
        self._end_blocks_to_dispense = 1
        self._last_cluster_info = None
        self._midgame_busy = False
        self._midgame_cycle_id = 0
        self._midgame_nav_attempt_id = 0
        self._midgame_active_nav_handle = None
        self._midgame_active_pick_handle = None
        self._midgame_active_dispense_handle = None
        self._midgame_current_cluster = None
        self._midgame_drop_candidates = []
        self._midgame_drop_candidate_index = 0
        self._midgame_drop_target = None
        self._midgame_last_pick_count = 0
        self._midgame_blocks_picked_count = 0
        self._midgame_flush_for_closer = False
        self._midgame_waiting_for_data_logged = False
        self._opener_started = False
        self._opener_finished = False
        self._opener_stage_index = 0
        self._opener_stages = self._build_opener_stages()
        self._opener_nav_wait_timer = None
        self._opener_retry_timer = None
        self._opener_retry_counts = {}
        self._opener_active_nav_handle = None
        self._opener_active_dock_handle = None
        self._opener_active_pick_handle = None
        self._opener_active_dispense_handle = None
        self._opener_active_therm_handle = None
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
            _delay = float(self.get_parameter('start_delay_s').value)
            if _delay > 0.0:
                self._start_delay_done = False
                self._start_delay_timer = self.create_timer(_delay, self._on_start_delay_done)
                self._publish_state()
                self._publish_team_color()
                self.get_logger().info(
                    f'force_running=true with start_delay_s={_delay:.1f}; '
                    'match will begin after delay.'
                )
            else:
                self._start_time = self.get_clock().now()
                self._running = True
                self._publish_state()
                self._publish_team_color()
                self.get_logger().info('Match running forced true for testing.')
                self._start_opener_if_needed()
        elif bool(self.get_parameter('autostart').value):
            self._begin_match()
        else:
            self._publish_state()
            self._publish_team_color()
            self.get_logger().info('Match node ready. Publish True to match/start to begin.')

    def _on_start(self, msg: Bool):
        if self._force_running:
            return

        if msg.data:
            if not self._running:
                self._begin_match()
        elif self._running:
            self._end_match('Match stopped.')

    def _on_start_delay_done(self):
        self._start_delay_timer.cancel()
        self._start_delay_timer = None
        self._start_delay_done = True
        self.get_logger().info('start_delay_s elapsed; beginning match now.')
        self._begin_match()
        self._publish_team_color()

    def _on_tick(self):
        if self._force_running:
            if not self._start_delay_done:
                self._publish_state()
                return
            if not self._running:
                self._running = True
            self._publish_state()
            self._start_opener_if_needed()
            self._start_end_if_needed()
            self._start_closer_if_needed()
            self._start_midgame_if_needed()
            return

        if not self._running or self._start_time is None:
            self._publish_activity_state()
            return

        self._publish_activity_state()
        self._start_end_if_needed()
        self._start_closer_if_needed()
        self._start_midgame_if_needed()

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
        self._end_finished = True
        self._cancel_midgame_active_goal_handles()
        self._cancel_opener_active_goal_handles()
        self._cancel_end_active_goal_handles()
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

    def _publish_team_color(self):
        msg = String()
        msg.data = self._team_color
        self._team_color_pub.publish(msg)

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

    def _start_midgame_if_needed(self):
        if (
            not self._enable_midgame
            or not self._running
            or self._midgame_busy
            or (self._opener_started and not self._opener_finished)
            or self._closer_started
            or self._closer_pending_after_midgame_flush
            or self._end_started
        ):
            return

        best_cluster = self._best_cluster_from_cluster_info()
        if best_cluster is None:
            if not self._midgame_waiting_for_data_logged:
                self.get_logger().info('Midgame waiting for /cluster_info best_cluster')
                self._midgame_waiting_for_data_logged = True
            return

        score = float(best_cluster.get('score', 0.0))
        if score < self._midgame_cluster_min_score:
            return

        if not self._ensure_manip_action_types():
            return

        self._midgame_waiting_for_data_logged = False
        self._start_midgame_cycle(best_cluster)

    def _best_cluster_from_cluster_info(self):
        if self._last_cluster_info is None:
            return None
        best_cluster = (
            self._last_cluster_info
            .get('step_2_selection', {})
            .get('best_cluster')
        )
        if isinstance(best_cluster, dict) and isinstance(best_cluster.get('center'), list):
            return best_cluster
        return None

    def _start_midgame_cycle(self, best_cluster):
        self._midgame_busy = True
        self._midgame_cycle_id += 1
        self._midgame_current_cluster = best_cluster
        self._midgame_drop_candidates = []
        self._midgame_drop_candidate_index = 0
        self._midgame_drop_target = None
        self._midgame_last_pick_count = 0
        self._midgame_blocks_picked_count = 0
        self._midgame_flush_for_closer = False

        center = best_cluster.get('center', [0.0, 0.0])
        zone_type = best_cluster.get('zone_type', 'free')
        zone_name = best_cluster.get('zone_name')
        self.get_logger().info(
            f'Midgame targeting cluster C{best_cluster.get("id", "?")} '
            f'in {zone_type}:{zone_name or "none"} at ({float(center[0]):.3f}, {float(center[1]):.3f})'
        )
        self._send_midgame_cluster_nav_goal(self._midgame_cycle_id)

    def _send_midgame_cluster_nav_goal(self, cycle_id: int):
        if not self._nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning('navigate_to_pose action server not available for midgame')
            self._finish_midgame_cycle()
            return

        cluster = self._midgame_current_cluster
        if cluster is None:
            self._finish_midgame_cycle()
            return

        x, y, yaw = self._midgame_cluster_approach_pose(cluster)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._pose_from_xy_yaw(x, y, yaw)
        self._midgame_nav_attempt_id += 1
        attempt_id = self._midgame_nav_attempt_id
        send_future = self._nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda future: self._on_midgame_nav_goal_response(
                future,
                cycle_id,
                attempt_id,
                'cluster',
            )
        )

    def _midgame_cluster_approach_pose(self, cluster):
        center = cluster.get('center', [0.0, 0.0])
        center_x = float(center[0])
        center_y = float(center[1])
        robot_xy = self._closer_robot_xy()
        if robot_xy is None:
            return center_x, center_y, 0.0

        dx = center_x - robot_xy[0]
        dy = center_y - robot_xy[1]
        dist = math.hypot(dx, dy)
        if dist <= 1e-6:
            return center_x, center_y, 0.0

        offset = min(max(0.0, self._midgame_cluster_approach_offset_m), max(0.0, dist - 0.02))
        goal_x = center_x - (dx / dist) * offset
        goal_y = center_y - (dy / dist) * offset
        yaw = math.atan2(center_y - goal_y, center_x - goal_x)
        return goal_x, goal_y, yaw

    def _on_midgame_nav_goal_response(self, future, cycle_id: int, attempt_id: int, goal_kind: str):
        if cycle_id != self._midgame_cycle_id or attempt_id != self._midgame_nav_attempt_id:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Midgame Nav2 goal request failed: {exc}')
            self._handle_midgame_nav_failure(goal_kind)
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Midgame Nav2 goal was rejected')
            self._handle_midgame_nav_failure(goal_kind)
            return

        self._midgame_active_nav_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_midgame_nav_result(result, cycle_id, attempt_id, goal_kind)
        )

    def _on_midgame_nav_result(self, future, cycle_id: int, attempt_id: int, goal_kind: str):
        if cycle_id != self._midgame_cycle_id or attempt_id != self._midgame_nav_attempt_id:
            return

        self._midgame_active_nav_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Midgame Nav2 result failed: {exc}')
            self._handle_midgame_nav_failure(goal_kind)
            return

        if not result or getattr(result, 'error_code', 0) != 0:
            self.get_logger().warning(
                f'Midgame Nav2 goal failed with error_code={getattr(result, "error_code", "unknown")}'
            )
            self._handle_midgame_nav_failure(goal_kind)
            return

        if goal_kind == 'cluster':
            self.get_logger().info('Midgame reached cluster; starting pickup')
            self._send_midgame_pick_goal(cycle_id)
            return

        self.get_logger().info('Midgame reached drop zone; starting dispense')
        self._send_midgame_dispense_goal(cycle_id)

    def _handle_midgame_nav_failure(self, goal_kind: str):
        if goal_kind == 'drop':
            self._midgame_drop_candidate_index += 1
            self._send_midgame_drop_nav_goal(self._midgame_cycle_id)
            return
        self._finish_midgame_cycle()

    def _send_midgame_pick_goal(self, cycle_id: int):
        if not self._pick_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('pick action server is not available for midgame')
            self._finish_midgame_cycle()
            return

        colors, count = self._build_pick_colors()
        if count == 0:
            self.get_logger().warning('No /best_pickup available for midgame; using single-block fallback')
            colors = [1, 0, 0, 0]
            count = 1

        pick_goal = self._pick_action_type.Goal()
        pick_goal.colors = colors
        pick_goal.count = count
        pick_goal.timeout_sec = self._opener_pick_timeout_s
        self._midgame_last_pick_count = count
        self.get_logger().info(f'Midgame sending Pick goal: colors={colors}, count={count}')
        send_future = self._pick_client.send_goal_async(pick_goal)
        send_future.add_done_callback(
            lambda future: self._on_midgame_pick_goal_response(future, cycle_id)
        )

    def _on_midgame_pick_goal_response(self, future, cycle_id: int):
        if cycle_id != self._midgame_cycle_id:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Midgame pick goal request failed: {exc}')
            self._finish_midgame_cycle()
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Midgame pick goal was rejected')
            self._finish_midgame_cycle()
            return

        self._midgame_active_pick_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_midgame_pick_result(result, cycle_id)
        )

    def _on_midgame_pick_result(self, future, cycle_id: int):
        if cycle_id != self._midgame_cycle_id:
            return

        self._midgame_active_pick_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Midgame pick result failed: {exc}')
            self._finish_midgame_cycle()
            return

        if not getattr(result, 'success', False):
            self.get_logger().warning(f'Midgame pickup failed: {getattr(result, "message", "")}')
            self._finish_midgame_cycle()
            return

        picked_count = int(getattr(result, 'count', 0))
        if picked_count <= 0:
            picked_count = self._midgame_last_pick_count
        self._midgame_blocks_picked_count = max(1, picked_count)
        self.get_logger().info(f'Midgame pickup succeeded: {getattr(result, "message", "")}')
        self._prepare_midgame_drop_candidates()
        self._send_midgame_drop_nav_goal(cycle_id)

    def _prepare_midgame_drop_candidates(self):
        target_zone = self._choose_midgame_drop_zone()
        if target_zone is None:
            self._midgame_drop_candidates = []
            self._midgame_drop_target = None
            return

        self._midgame_drop_target = target_zone.get('name')
        candidates = self._drop_zone_goal_poses(target_zone)
        robot_xy = self._closer_robot_xy()
        if robot_xy is not None:
            candidates.sort(
                key=lambda candidate: math.hypot(candidate['x'] - robot_xy[0], candidate['y'] - robot_xy[1])
            )
        self._midgame_drop_candidates = candidates
        self._midgame_drop_candidate_index = 0

    def _prepare_midgame_flush_drop_candidates(self):
        target_zone = self._choose_closest_garde_manger_zone()
        if target_zone is None:
            self._midgame_drop_candidates = []
            self._midgame_drop_target = None
            return

        self._midgame_drop_target = target_zone.get('name')
        candidates = self._drop_zone_goal_poses(target_zone)
        robot_xy = self._closer_robot_xy()
        if robot_xy is not None:
            candidates.sort(
                key=lambda candidate: math.hypot(candidate['x'] - robot_xy[0], candidate['y'] - robot_xy[1])
            )
        self._midgame_drop_candidates = candidates
        self._midgame_drop_candidate_index = 0
        self.get_logger().info(
            f'Midgame flush before closer selected {self._midgame_drop_target}'
        )

    def _choose_midgame_drop_zone(self):
        summaries = self._last_cluster_info.get('garde_manger_summary', []) if self._last_cluster_info else []
        cluster = self._midgame_current_cluster or {}
        zone_type = cluster.get('zone_type', 'free')
        zone_name = cluster.get('zone_name')
        if zone_type == 'garde_manger' and zone_name:
            for zone in summaries:
                if zone.get('name') == zone_name:
                    return zone

        if not summaries:
            return None

        center = cluster.get('center', [0.0, 0.0])
        center_x = float(center[0])
        center_y = float(center[1])
        return min(
            summaries,
            key=lambda zone: (
                int(zone.get('total_count', 0)),
                math.hypot(float(zone.get('center', [center_x, center_y])[0]) - center_x,
                           float(zone.get('center', [center_x, center_y])[1]) - center_y),
            ),
        )

    def _choose_closest_garde_manger_zone(self):
        summaries = self._last_cluster_info.get('garde_manger_summary', []) if self._last_cluster_info else []
        if not summaries:
            return None

        robot_xy = self._closer_robot_xy()
        if robot_xy is None:
            cluster = self._midgame_current_cluster or {}
            center = cluster.get('center', [0.0, 0.0])
            robot_xy = (float(center[0]), float(center[1]))

        return min(
            summaries,
            key=lambda zone: math.hypot(
                float(zone.get('center', [robot_xy[0], robot_xy[1]])[0]) - robot_xy[0],
                float(zone.get('center', [robot_xy[0], robot_xy[1]])[1]) - robot_xy[1],
            ),
        )

    def _drop_zone_goal_poses(self, zone):
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
        zone_name = zone.get('name', 'garde_manger')

        raw_candidates = [
            {
                'zone_name': zone_name,
                'x': x_min - OPENER_ROBOT_REAR_EXTENT_M,
                'y': center_y,
                'yaw': math.pi,
                'side': 'left',
            },
            {
                'zone_name': zone_name,
                'x': x_max + OPENER_ROBOT_REAR_EXTENT_M,
                'y': center_y,
                'yaw': 0.0,
                'side': 'right',
            },
            {
                'zone_name': zone_name,
                'x': center_x,
                'y': y_min - OPENER_ROBOT_REAR_EXTENT_M,
                'yaw': -math.pi / 2.0,
                'side': 'bottom',
            },
            {
                'zone_name': zone_name,
                'x': center_x,
                'y': y_max + OPENER_ROBOT_REAR_EXTENT_M,
                'yaw': math.pi / 2.0,
                'side': 'top',
            },
        ]
        return [
            candidate for candidate in raw_candidates
            if self._closer_pose_fits_table(candidate['x'], candidate['y'], candidate['yaw'])
        ]

    def _send_midgame_drop_nav_goal(self, cycle_id: int):
        if cycle_id != self._midgame_cycle_id:
            return

        if self._midgame_drop_candidate_index >= len(self._midgame_drop_candidates):
            self.get_logger().warning('Midgame found no usable outside drop pose')
            self._finish_midgame_cycle()
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning('navigate_to_pose action server not available for midgame drop')
            self._finish_midgame_cycle()
            return

        candidate = self._midgame_drop_candidates[self._midgame_drop_candidate_index]
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._pose_from_xy_yaw(candidate['x'], candidate['y'], candidate['yaw'])
        self._midgame_nav_attempt_id += 1
        attempt_id = self._midgame_nav_attempt_id
        self.get_logger().info(
            f'Midgame dropping at {candidate["zone_name"]} from {candidate["side"]} side '
            f'({candidate["x"]:.3f}, {candidate["y"]:.3f})'
        )
        send_future = self._nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda future: self._on_midgame_nav_goal_response(
                future,
                cycle_id,
                attempt_id,
                'drop',
            )
        )

    def _send_midgame_dispense_goal(self, cycle_id: int):
        if not self._dispense_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('dispense action server is not available for midgame')
            self._finish_midgame_cycle()
            return

        dispense_goal = self._dispense_action_type.Goal()
        dispense_goal.count = max(1, self._midgame_blocks_picked_count)
        dispense_goal.timeout_sec = self._opener_dispense_timeout_s
        self.get_logger().info(f'Midgame sending Dispense goal: count={dispense_goal.count}')
        send_future = self._dispense_client.send_goal_async(dispense_goal)
        send_future.add_done_callback(
            lambda future: self._on_midgame_dispense_goal_response(future, cycle_id)
        )

    def _on_midgame_dispense_goal_response(self, future, cycle_id: int):
        if cycle_id != self._midgame_cycle_id:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Midgame dispense goal request failed: {exc}')
            self._finish_midgame_cycle()
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Midgame dispense goal was rejected')
            self._finish_midgame_cycle()
            return

        self._midgame_active_dispense_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_midgame_dispense_result(result, cycle_id)
        )

    def _on_midgame_dispense_result(self, future, cycle_id: int):
        if cycle_id != self._midgame_cycle_id:
            return

        self._midgame_active_dispense_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'Midgame dispense result failed: {exc}')
            self._finish_midgame_cycle()
            return

        if not getattr(result, 'success', False):
            self.get_logger().warning(f'Midgame dispense failed: {getattr(result, "message", "")}')
            self._finish_midgame_cycle()
            return

        self.get_logger().info(f'Midgame cycle complete: {getattr(result, "message", "")}')
        self._finish_midgame_cycle()

    def _finish_midgame_cycle(self):
        start_pending_closer = (
            self._closer_pending_after_midgame_flush
            and not self._closer_started
            and not self._end_started
            and self._running
            and self._enable_closer
        )
        was_flush = self._midgame_flush_for_closer

        self._midgame_busy = False
        self._midgame_active_nav_handle = None
        self._midgame_active_pick_handle = None
        self._midgame_active_dispense_handle = None
        self._midgame_current_cluster = None
        self._midgame_drop_candidates = []
        self._midgame_drop_candidate_index = 0
        self._midgame_drop_target = None
        self._midgame_blocks_picked_count = 0
        self._midgame_flush_for_closer = False
        self._closer_pending_after_midgame_flush = False

        if start_pending_closer:
            if was_flush:
                self.get_logger().info('Midgame flush complete; starting closer')
            else:
                self.get_logger().warning('Midgame flush did not complete cleanly; starting closer anyway')
            self.closer()

    def _cancel_midgame_active_goal_handles(self):
        self._midgame_cycle_id += 1
        for goal_handle in (
            self._midgame_active_nav_handle,
            self._midgame_active_pick_handle,
            self._midgame_active_dispense_handle,
        ):
            if goal_handle is None:
                continue
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Could not cancel midgame goal: {exc}')
        self._finish_midgame_cycle()

    def _start_midgame_flush_before_closer(self) -> bool:
        if self._midgame_blocks_picked_count <= 0:
            return False

        if self._closer_pending_after_midgame_flush:
            return True

        self._closer_pending_after_midgame_flush = True
        self._midgame_flush_for_closer = True
        self._midgame_busy = True

        if self._midgame_active_dispense_handle is not None:
            self.get_logger().info(
                'Closer is waiting for midgame dispense to finish before starting'
            )
            return True

        self._midgame_cycle_id += 1
        for goal_handle in (
            self._midgame_active_nav_handle,
            self._midgame_active_pick_handle,
        ):
            if goal_handle is None:
                continue
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Could not cancel midgame goal for closer flush: {exc}')

        self._midgame_active_nav_handle = None
        self._midgame_active_pick_handle = None
        self._prepare_midgame_flush_drop_candidates()

        if not self._midgame_drop_candidates:
            self.get_logger().warning(
                'Midgame has undispensed blocks but no garde-manger flush pose was available'
            )
            self._finish_midgame_cycle()
            return True

        self.get_logger().info(
            'Closer start delayed: flushing midgame blocks to closest garde-manger first'
        )
        self._send_midgame_drop_nav_goal(self._midgame_cycle_id)
        return True

    def _start_end_if_needed(self):
        if (
            not self._enable_end
            or not self._running
            or self._end_started
            or self._end_finished
            or self._start_time is None
        ):
            return

        elapsed = self.get_clock().now() - self._start_time
        end_start_time = self._duration - self._end_before_end
        if elapsed >= end_start_time:
            self.end('final timer')

    def end(self, reason: str = 'requested'):
        if not self._enable_end:
            self.get_logger().info(f'End skipped ({reason}) because enable_end is false')
            return

        if self._end_started:
            return

        self._end_started = True
        self._end_finished = False
        self._opener_finished = True
        self._closer_finished = True
        self._closer_pending_after_midgame_flush = False
        self._midgame_flush_for_closer = False
        self._end_blocks_to_dispense = max(1, self._closer_blocks_picked_count)
        self._cancel_midgame_active_goal_handles()
        self._cancel_opener_active_goal_handles()
        self._cancel_closer_plan_timeout()
        self._cancel_closer_active_goal_handles()
        self._publish_activity_state()

        if not self._ensure_manip_action_types():
            self._finish_end('End stopped because action types are unavailable')
            return

        self.get_logger().info(
            f'End started ({reason}): returning to team nid for final dispense'
        )
        self._send_end_nid_goal()

    def _send_end_nid_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=0.2):
            self._finish_end('End stopped because Nav2 is unavailable for team nid return')
            return

        x, y, yaw = self._team_nid_pose()
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._pose_from_xy_yaw(x, y, yaw)
        self.get_logger().info(f'End returning to team nid at ({x:.3f}, {y:.3f})')
        self._end_nav_attempt_id += 1
        attempt_id = self._end_nav_attempt_id
        send_future = self._nav_client.send_goal_async(nav_goal)
        send_future.add_done_callback(
            lambda future: self._on_end_nav_goal_response(future, attempt_id)
        )

    def _on_end_nav_goal_response(self, future, attempt_id: int):
        if attempt_id != self._end_nav_attempt_id or self._end_finished:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_end(f'End Nav2 goal request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._finish_end('End Nav2 goal was rejected')
            return

        self._end_active_nav_handle = goal_handle
        self.get_logger().info('End Nav2 goal accepted for team nid')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_end_nav_result(result, attempt_id)
        )

    def _on_end_nav_result(self, future, attempt_id: int):
        if attempt_id != self._end_nav_attempt_id or self._end_finished:
            return

        self._end_active_nav_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self._finish_end(f'End Nav2 result failed: {exc}')
            return

        if not result or getattr(result, 'error_code', 0) != 0:
            self._finish_end(
                f'End Nav2 goal failed with error_code={getattr(result, "error_code", "unknown")}'
            )
            return

        self.get_logger().info('End reached team nid; starting final dispense')
        self._send_end_dispense_goal()

    def _team_nid_pose(self):
        if self._team_color == 'yellow':
            return 0.300, 1.775, 0.0
        return 2.700, 1.775, math.pi

    def _send_end_dispense_goal(self):
        if not self._dispense_client.wait_for_server(timeout_sec=2.0):
            self._finish_end('End stopped because dispense action server is unavailable')
            return

        dispense_goal = self._dispense_action_type.Goal()
        dispense_goal.count = max(1, self._end_blocks_to_dispense)
        dispense_goal.timeout_sec = self._opener_dispense_timeout_s
        self.get_logger().info(f'End sending Dispense goal: count={dispense_goal.count}')
        send_future = self._dispense_client.send_goal_async(dispense_goal)
        send_future.add_done_callback(self._on_end_dispense_goal_response)

    def _on_end_dispense_goal_response(self, future):
        if self._end_finished:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_end(f'End dispense goal request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._finish_end('End dispense goal was rejected')
            return

        self._end_active_dispense_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_end_dispense_result)

    def _on_end_dispense_result(self, future):
        if self._end_finished:
            return

        self._end_active_dispense_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self._finish_end(f'End dispense result failed: {exc}')
            return

        if not getattr(result, 'success', False):
            self._finish_end(f'End dispense failed: {getattr(result, "message", "")}')
            return

        self._finish_end(f'End complete: dropped blocks in team nid ({getattr(result, "message", "")})')

    def _finish_end(self, log_message: str):
        self._end_finished = True
        self._end_active_nav_handle = None
        self._end_active_dispense_handle = None
        self._publish_activity_state()
        self.get_logger().info(log_message)

    def _cancel_end_active_goal_handles(self):
        self._end_nav_attempt_id += 1
        for goal_handle in (
            self._end_active_nav_handle,
            self._end_active_dispense_handle,
        ):
            if goal_handle is None:
                continue
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Could not cancel end goal: {exc}')
        self._end_active_nav_handle = None
        self._end_active_dispense_handle = None

    def _start_closer_if_needed(self):
        if (
            not self._enable_closer
            or not self._running
            or self._closer_started
            or self._closer_finished
            or self._closer_pending_after_midgame_flush
            or self._end_started
            or (self._opener_started and not self._opener_finished)
            or self._start_time is None
        ):
            return

        elapsed = self.get_clock().now() - self._start_time
        closer_start_time = self._duration - self._closer_start_before_end
        if elapsed >= closer_start_time:
            if self._start_midgame_flush_before_closer():
                return
            self.closer()

    def closer(self):
        self._closer_started = True
        self._cancel_midgame_active_goal_handles()
        self._publish_activity_state()
        if not self._ensure_manip_action_types():
            self._closer_finished = True
            self._publish_activity_state()
            return

        self._closer_goal_candidates = self._build_closer_goal_candidates()
        self._closer_candidate_index = 0
        self._closer_current_zone_name = None
        self._closer_zone_deadline_time = 0.0
        self._closer_successful_pickups = 0
        self._closer_blocks_picked_count = 0
        self._closer_last_pick_count = 0
        if not self._closer_goal_candidates:
            self._closer_finished = True
            self._publish_activity_state()
            self.get_logger().warning('Closer found no garde-manger containing enemy blocks')
            self.end('closer completed with no enemy blocks')
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

        candidate = self._closer_goal_candidates[self._closer_candidate_index]
        self.get_logger().info(f'Closer reached {candidate["zone_name"]}; starting pickup action')
        self._send_closer_pick_goal()

    def _handle_closer_nav_failure(self, goal_kind: str):
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

        if self._end_started:
            goal_handle.cancel_goal_async()
            return

        self._closer_active_pick_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_closer_pick_result)

    def _on_closer_pick_result(self, future):
        if self._end_started:
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
            self._finish_closer('Closer completed pickups; handing off to end')
            self.end('closer completed')
            return

        self._restart_closer_selection()

    def _cancel_closer_active_goal_handles(self):
        for goal_handle in (
            self._closer_active_plan_handle,
            self._closer_active_nav_handle,
            self._closer_active_pick_handle,
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
            self.end('closer completed')
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
        self._cancel_midgame_active_goal_handles()
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
            f'{failure_message}; opener holding active after {self._opener_step_retries} retries'
        )
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

    def _cancel_opener_active_goal_handles(self):
        if self._opener_nav_wait_timer is not None:
            self._opener_nav_wait_timer.cancel()
            self._opener_nav_wait_timer = None
        if self._opener_retry_timer is not None:
            self._opener_retry_timer.cancel()
            self._opener_retry_timer = None

        for goal_handle in (
            self._opener_active_nav_handle,
            self._opener_active_dock_handle,
            self._opener_active_pick_handle,
            self._opener_active_dispense_handle,
            self._opener_active_therm_handle,
        ):
            if goal_handle is None:
                continue
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Could not cancel opener goal: {exc}')

        self._opener_active_nav_handle = None
        self._opener_active_dock_handle = None
        self._opener_active_pick_handle = None
        self._opener_active_dispense_handle = None
        self._opener_active_therm_handle = None

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
        stock_3_left_edge_x = 1.000
        stock_7_right_edge_x = 2.000
        bottom_stock_goal_y = 0.175
        garde_manger_8_inner_edge_x = 0.800
        garde_manger_10_inner_edge_x = 2.200
        garde_manger_4_goal_x = 0.800
        garde_manger_6_goal_x = 2.200
        lower_garde_manger_goal_y = lower_garde_manger_back_y - OPENER_ROBOT_REAR_EXTENT_M
        stock_3_goal_x = stock_3_left_edge_x - OPENER_STOCK_CLEARANCE_M - OPENER_ROBOT_FRONT_EXTENT_M
        stock_7_goal_x = stock_7_right_edge_x + OPENER_STOCK_CLEARANCE_M + OPENER_ROBOT_FRONT_EXTENT_M
        garde_manger_8_inner_goal_x = garde_manger_8_inner_edge_x + OPENER_ROBOT_REAR_EXTENT_M
        garde_manger_10_inner_goal_x = garde_manger_10_inner_edge_x - OPENER_ROBOT_REAR_EXTENT_M

        if self._team_color == 'yellow':
            return [
                # Front edge 10 cm above stock_1's y=1.300 edge, facing -y.
                {'kind': 'stock', 'name': 'stock_1', 'x': left_stock_goal_x, 'y': upper_stock_goal_y, 'yaw': -math.pi / 2.0},
                # Front edge 10 cm above stock_2's y=0.500 edge, facing -y.
                {'kind': 'stock', 'name': 'stock_2', 'x': left_stock_goal_x, 'y': lower_stock_goal_y, 'yaw': -math.pi / 2.0},
                # Rear edge lined up with garde_manger_3's lower y edge.
                {'kind': 'drop', 'name': 'garde_manger_3', 'x': left_wall_garde_manger_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
                {'kind': 'therm', 'name': 'therm'},
                # Front edge 10 cm left of stock_3's x=1.000 edge, facing +x.
                {'kind': 'stock', 'name': 'stock_3', 'x': stock_3_goal_x, 'y': bottom_stock_goal_y, 'yaw': 0.0},
                # Drive forward; rear edge lined up with garde_manger_8's x=0.800 inner edge.
                {'kind': 'drop', 'name': 'garde_manger_8', 'x': garde_manger_8_inner_goal_x, 'y': 0.100, 'yaw': 0.0},
                # Rear edge lined up with garde_manger_4's lower y edge, facing -y.
                {'kind': 'drop', 'name': 'garde_manger_4', 'x': garde_manger_4_goal_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
            ]

        return [
            # Front edge 10 cm above stock_5's y=1.300 edge, facing -y.
            {'kind': 'stock', 'name': 'stock_5', 'x': right_stock_goal_x, 'y': upper_stock_goal_y, 'yaw': -math.pi / 2.0},
            # Front edge 10 cm above stock_6's y=0.500 edge, facing -y.
            {'kind': 'stock', 'name': 'stock_6', 'x': right_stock_goal_x, 'y': lower_stock_goal_y, 'yaw': -math.pi / 2.0},
            # Rear edge lined up with garde_manger_7's lower y edge.
            {'kind': 'drop', 'name': 'garde_manger_7', 'x': right_wall_garde_manger_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
            {'kind': 'therm', 'name': 'therm'},
            # Front edge 10 cm right of stock_7's x=2.000 edge, facing -x.
            {'kind': 'stock', 'name': 'stock_7', 'x': stock_7_goal_x, 'y': bottom_stock_goal_y, 'yaw': math.pi},
            # Drive forward; rear edge lined up with garde_manger_10's x=2.200 inner edge.
            {'kind': 'drop', 'name': 'garde_manger_10', 'x': garde_manger_10_inner_goal_x, 'y': 0.100, 'yaw': math.pi},
            # Rear edge lined up with garde_manger_6's lower y edge, facing -y.
            {'kind': 'drop', 'name': 'garde_manger_6', 'x': garde_manger_6_goal_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
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

    def _cancel_late_opener_goal_if_finished(self, future) -> bool:
        if not self._opener_finished:
            return False

        try:
            goal_handle = future.result()
        except Exception:
            return True

        try:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warning(f'Could not cancel late opener goal: {exc}')
        return True

    def _on_nav_goal_response(self, future):
        if self._cancel_late_opener_goal_if_finished(future):
            return

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
        self._opener_active_nav_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        if self._opener_finished:
            self._opener_active_nav_handle = None
            return

        self._opener_active_nav_handle = None
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
        if self._cancel_late_opener_goal_if_finished(future):
            return

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
        self._opener_active_dock_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dock_result)

    def _on_dock_result(self, future):
        if self._opener_finished:
            self._opener_active_dock_handle = None
            return

        self._opener_active_dock_handle = None
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
        if self._cancel_late_opener_goal_if_finished(future):
            return

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

        self._opener_active_pick_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_pick_result)

    def _on_pick_result(self, future):
        if self._opener_finished:
            self._opener_active_pick_handle = None
            return

        self._opener_active_pick_handle = None
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
        if self._cancel_late_opener_goal_if_finished(future):
            return

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

        self._opener_active_dispense_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dispense_result)

    def _on_dispense_result(self, future):
        if self._opener_finished:
            self._opener_active_dispense_handle = None
            return

        self._opener_active_dispense_handle = None
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
        if self._cancel_late_opener_goal_if_finished(future):
            return

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

        self._opener_active_therm_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_therm_result)

    def _on_therm_result(self, future):
        if self._opener_finished:
            self._opener_active_therm_handle = None
            return

        self._opener_active_therm_handle = None
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
