import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


MATCH_DURATION = 100.0  # seconds


class MatchTimerNode(Node):

    def __init__(self):
        super().__init__('match_timer')

        self._time_pub = self.create_publisher(Float32, 'match/time_remaining', 10)
        self._running_pub = self.create_publisher(Bool, 'match/running', 10)

        self.declare_parameter('autostart', False)

        self._remaining = MATCH_DURATION
        self._running = False

        self._tick = self.create_timer(0.1, self._on_tick)

        self.get_logger().info('Match timer ready. Publish True to match/start to begin.')

        self.create_subscription(Bool, 'match/start', self._on_start, 10)

    def _on_start(self, msg: Bool):
        if msg.data and not self._running:
            self._remaining = MATCH_DURATION
            self._running = True
            self.get_logger().info('Match started — counting down from 100 s.')
        elif not msg.data and self._running:
            self._running = False
            self.get_logger().info('Match stopped.')

    def _on_tick(self):
        if self._running:
            self._remaining = max(0.0, self._remaining - 0.1)
            if self._remaining == 0.0:
                self._running = False
                self.get_logger().info('Match over.')

        time_msg = Float32()
        time_msg.data = self._remaining
        self._time_pub.publish(time_msg)

        running_msg = Bool()
        running_msg.data = self._running
        self._running_pub.publish(running_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MatchTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
