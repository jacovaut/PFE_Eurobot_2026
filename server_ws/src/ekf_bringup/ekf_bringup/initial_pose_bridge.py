#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from robot_localization.srv import SetPose


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__("initial_pose_bridge")

        self.input_topic = self.declare_parameter("input_topic", "/initialpose").value
        self.set_pose_service = self.declare_parameter(
            "set_pose_service",
            "/set_pose",
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value

        self.client = self.create_client(SetPose, self.set_pose_service)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.initial_pose_cb,
            10,
        )

        self.get_logger().info(
            f"Forwarding {self.input_topic} to {self.set_pose_service}"
        )

    def initial_pose_cb(self, msg):
        frame_id = msg.header.frame_id or self.map_frame
        if frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring initial pose in frame '{frame_id}'. "
                f"Set RViz Fixed Frame to '{self.map_frame}'."
            )
            return

        if not self.client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                f"SetPose service {self.set_pose_service} is not available yet."
            )
            return

        request = SetPose.Request()
        request.pose = msg
        request.pose.header.frame_id = self.map_frame
        if request.pose.header.stamp.sec == 0 and request.pose.header.stamp.nanosec == 0:
            request.pose.header.stamp = self.get_clock().now().to_msg()

        yaw_deg = math.degrees(yaw_from_quaternion(request.pose.pose.pose.orientation))
        self.get_logger().info(
            "Setting global EKF pose from RViz: "
            f"x={request.pose.pose.pose.position.x:.3f} "
            f"y={request.pose.pose.pose.position.y:.3f} "
            f"yaw={yaw_deg:.1f} deg"
        )

        future = self.client.call_async(request)
        future.add_done_callback(self.set_pose_done)

    def set_pose_done(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"SetPose call failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
