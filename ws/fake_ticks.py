#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from deadwheel_msgs.msg import DeadwheelTicks

class FakeTicks(Node):
    def __init__(self):
        super().__init__('fake_deadwheel_ticks')
        self.pub = self.create_publisher(DeadwheelTicks, 'deadwheel_ticks', 10)

        self.t0 = 0
        self.t1 = 0
        self.t2 = 0

        self.timer = self.create_timer(1.0/50.0, self.tick)

    def tick(self):
        msg = DeadwheelTicks()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Simulate straight motion: both main wheels advance equally
        self.t0 += 20
        self.t1 += 20
        self.t2 += 0

        msg.t0 = self.t0
        msg.t1 = self.t1
        msg.t2 = self.t2

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeTicks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
