import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


MATCH_DURATION = 98.0  # seconds


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

        self._duration = Duration(seconds=float(self.get_parameter('duration_s').value))
        self._start_time = None
        self._running = False

        self._tick = self.create_timer(0.1, self._on_tick)

        self.create_subscription(Bool, 'match/start', self._on_start, 10)

        self._publish_state()

        if bool(self.get_parameter('autostart').value):
            self._begin_match()
        else:
            self.get_logger().info('Match node ready. Publish True to match/start to begin.')

    def _on_start(self, msg: Bool):
        if msg.data:
            if not self._running:
                self._begin_match()
        elif self._running:
            self._end_match('Match stopped.')

    def _on_tick(self):
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

    def _end_match(self, log_message: str):
        self._running = False
        self._publish_state()
        self.get_logger().info(log_message)

    def _publish_state(self):
        running_msg = Bool()
        running_msg.data = self._running
        self._running_pub.publish(running_msg)


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
