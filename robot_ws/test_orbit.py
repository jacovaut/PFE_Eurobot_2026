#!/usr/bin/env python3
"""
Test orbit: drives the robot in a circle of 20 cm radius.

Usage:
    python3 test_orbit.py [--radius 0.20] [--speed 0.12] [--turns 1]

The robot is holonomic. To orbit at radius R with tangential speed v,
the velocity vector rotates at ω = v/R:
    vx(t) = -v * sin(ωt)
    vy(t) =  v * cos(ωt)   (then cmd.linear.y = -vy per robot convention)
    w(t)  = 0               (robot does not rotate)
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=0.50, help="Orbit radius in metres")
    parser.add_argument("--speed",  type=float, default=0.12, help="Tangential speed in m/s")
    parser.add_argument("--turns",  type=float, default=1.0,  help="Number of full turns")
    args = parser.parse_args()

    R   = args.radius
    v   = args.speed
    omega   = v / R                          # rad/s
    period  = 2.0 * math.pi / omega         # seconds per turn
    duration = period * args.turns

    rclpy.init()
    node = Node("test_orbit")
    pub  = node.create_publisher(Twist, "/cmd_vel_smoothed", 10)

    node.get_logger().info(
        f"[TEST ORBIT] R={R*100:.0f}cm  v={v:.2f}m/s  "
        f"ω={math.degrees(omega):.1f}°/s  period={period:.1f}s  "
        f"duration={duration:.1f}s"
    )

    rate_hz = 50.0
    dt      = 1.0 / rate_hz
    t_start = time.time()

    try:
        while rclpy.ok():
            t = time.time() - t_start
            if t >= duration:
                break

            # In the robot's rotating frame, orbiting while facing the center
            # is just constant sideways sliding. The angular velocity sweeps
            # that local-frame velocity around the circle in world frame.
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.linear.y  = v   # slides in local -y; convention: driver uses cmd.linear.y directly
            cmd.angular.z = -omega
            pub.publish(cmd)

            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(dt)

    finally:
        # Stop the robot
        pub.publish(Twist())
        node.get_logger().info("[TEST ORBIT] Done — robot stopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
