import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


HELP_TEXT = """
PAMI Ninja ROS keyboard control

Movement:
  W/S - forward/backward
  A/D - strafe left/right
  Q/E - rotate counter-clockwise/clockwise
  Space - stop

Block handling:
  P - pick up block
  R - release block

Servo:
  U - raise servos
  O - lower servos

Pump / encoders / speed:
  J - toggle pump
  N - toggle encoder print
  L - reset encoders
  + - increase speed
  - - decrease speed
  ] - increase omega speed
  [ - decrease omega speed

Movement keys keep publishing until another movement key or Space is pressed.
Ctrl-C to quit.
"""


class PamiKeyboard(Node):
    def __init__(self):
        super().__init__("pami_keyboard")
        self.key_pub = self.create_publisher(String, "pami_ninja/keyboard", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_smoothed", 10)
        self.valid_keys = set("WSADQE PRUOJNL+-[]")
        self.movement_keys = set("WSADQE")
        self.active_movement_key = None
        self.linear_speed = 0.39
        self.angular_speed = 0.35
        self.angular_speed_step = 0.05
        self.min_angular_speed = 0.05
        self.max_angular_speed = 1.00
        self.get_logger().info(
            "Publishing keys on /pami_ninja/keyboard and movement on /cmd_vel_match_input"
        )

    def publish_key(self, key: str, show: bool = False):
        msg = String()
        msg.data = key
        self.key_pub.publish(msg)
        if show:
            shown_key = "SPACE" if key == " " else key
            print(f"\rSent: {shown_key:<5}", end="", flush=True)

    def publish_twist(self, key: str):
        msg = Twist()

        if key == "W":
            msg.linear.y = self.linear_speed
        elif key == "S":
            msg.linear.y = -self.linear_speed
        elif key == "A":
            msg.linear.x = self.linear_speed
        elif key == "D":
            msg.linear.x = -self.linear_speed
        elif key == "Q":
            msg.angular.z = self.angular_speed
        elif key == "E":
            msg.angular.z = -self.angular_speed

        self.cmd_pub.publish(msg)

    def handle_keypress(self, key: str):
        if key == "\x03":
            raise KeyboardInterrupt

        key = key.upper()
        if key not in self.valid_keys:
            return

        if key in self.movement_keys:
            self.active_movement_key = key
        elif key == " ":
            self.active_movement_key = None
        elif key == "]":
            self.angular_speed = min(
                self.angular_speed + self.angular_speed_step,
                self.max_angular_speed,
            )
            print(f"\rOmega speed: {self.angular_speed:.2f} rad/s", end="", flush=True)
        elif key == "[":
            self.angular_speed = max(
                self.angular_speed - self.angular_speed_step,
                self.min_angular_speed,
            )
            print(f"\rOmega speed: {self.angular_speed:.2f} rad/s", end="", flush=True)

        self.publish_key(key, show=(key not in "[]"))
        if key in self.movement_keys or key == " ":
            self.publish_twist(key)


def main(args=None):
    rclpy.init(args=args)
    node = PamiKeyboard()

    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP_TEXT)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                node.handle_keypress(sys.stdin.read(1))
            elif node.active_movement_key is not None:
                node.publish_key(node.active_movement_key)
                node.publish_twist(node.active_movement_key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
