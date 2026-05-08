import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
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

        self.declare_parameter('autostart', False)
        self.declare_parameter('duration_s', MATCH_DURATION)
        self.declare_parameter('force_running', False)
        self.declare_parameter('team_color', 'blue')
        self.declare_parameter('enable_opener', False)
        self.declare_parameter('opener_nav_action_name', 'navigate_to_pose')
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

        self._duration = Duration(seconds=float(self.get_parameter('duration_s').value))
        self._force_running = bool(self.get_parameter('force_running').value)
        self._team_color = self._normalize_team_color(self.get_parameter('team_color').value)
        self._enable_opener = bool(self.get_parameter('enable_opener').value)
        self._opener_dock_timeout_s = float(self.get_parameter('opener_dock_timeout_s').value)
        self._opener_pick_timeout_s = float(self.get_parameter('opener_pick_timeout_s').value)
        self._opener_dispense_timeout_s = float(self.get_parameter('opener_dispense_timeout_s').value)
        self._opener_therm_timeout_s = float(self.get_parameter('opener_therm_timeout_s').value)
        self._opener_dispense_count = int(self.get_parameter('opener_dispense_count').value)
        self._opener_therm_direction = self.get_parameter('opener_therm_direction').value
        self._opener_step_retries = int(self.get_parameter('opener_step_retries').value)
        self._start_time = None
        self._running = False
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
        self._dock_client = None
        self._pick_client = None
        self._dispense_client = None
        self._therm_client = None

        self._tick = self.create_timer(0.1, self._on_tick)

        self.create_subscription(Bool, 'match/start', self._on_start, 10)
        self.create_subscription(String, '/best_pickup', self._on_best_pickup, 10)

        if self._force_running:
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
            return

        if not self._running or self._start_time is None:
            return

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
        self._publish_state()
        self.get_logger().info(log_message)

    def _publish_state(self):
        running_msg = Bool()
        running_msg.data = self._running
        self._running_pub.publish(running_msg)

    def _on_best_pickup(self, msg: String):
        try:
            self._last_best_pickup = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Could not parse /best_pickup JSON for opener pick goal')

    def _start_opener_if_needed(self):
        if not self._enable_opener or not self._running or self._opener_started:
            return

        self._opener_started = True
        if not self._ensure_manip_action_types():
            self._opener_finished = True
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
        left_stock_goal_x = 0.250 + OPENER_STOCK_CLEARANCE_M + OPENER_ROBOT_FRONT_EXTENT_M
        right_stock_goal_x = 2.750 - OPENER_STOCK_CLEARANCE_M - OPENER_ROBOT_FRONT_EXTENT_M
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
                # Robot body 10 cm outside stock_1 on the table-interior side.
                {'kind': 'stock', 'name': 'stock_1', 'x': left_stock_goal_x, 'y': 1.200, 'yaw': math.pi},
                # Robot body 10 cm outside stock_2 on the table-interior side.
                {'kind': 'stock', 'name': 'stock_2', 'x': left_stock_goal_x, 'y': 0.400, 'yaw': math.pi},
                # Rear edge lined up with garde_manger_3's lower y edge.
                {'kind': 'drop', 'name': 'garde_manger_3', 'x': left_wall_garde_manger_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
                {'kind': 'therm', 'name': 'therm'},
                # Back side lined up with garde_manger_8's x=0.600 edge, facing -x.
                {'kind': 'drop', 'name': 'garde_manger_8', 'x': garde_manger_8_goal_x, 'y': 0.100, 'yaw': math.pi},
            ]

        return [
            # Robot body 10 cm outside stock_5 on the table-interior side.
            {'kind': 'stock', 'name': 'stock_5', 'x': right_stock_goal_x, 'y': 1.200, 'yaw': 0.0},
            # Robot body 10 cm outside stock_6 on the table-interior side.
            {'kind': 'stock', 'name': 'stock_6', 'x': right_stock_goal_x, 'y': 0.400, 'yaw': 0.0},
            # Rear edge lined up with garde_manger_7's lower y edge.
            {'kind': 'drop', 'name': 'garde_manger_7', 'x': right_wall_garde_manger_x, 'y': lower_garde_manger_goal_y, 'yaw': -math.pi / 2.0},
            {'kind': 'therm', 'name': 'therm'},
            # Back side lined up with garde_manger_10's x=2.400 edge, facing +x.
            {'kind': 'drop', 'name': 'garde_manger_10', 'x': garde_manger_10_goal_x, 'y': 0.100, 'yaw': 0.0},
        ]

    def _opener_goal_pose(self, stage) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = stage['x']
        pose.pose.position.y = stage['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(stage['yaw'] * 0.5)
        pose.pose.orientation.w = math.cos(stage['yaw'] * 0.5)
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
