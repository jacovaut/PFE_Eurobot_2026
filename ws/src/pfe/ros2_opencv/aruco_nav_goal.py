import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class ArucoNavGoal(Node):
    def __init__(self):
        super().__init__('aruco_nav_goal')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.target_frame = "aruco_47"   # marker frame
        self.fixed_frame = "world"       # YOUR map frame

        self.goal_sent = False
        self.timer = self.create_timer(1.0, self.try_send_goal)

    def try_send_goal(self):
        if self.goal_sent:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.target_frame,
                rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header.frame_id = self.fixed_frame
            pose.header.stamp = self.get_clock().now().to_msg()

            # block position in world frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0

            pose.pose.orientation = transform.transform.rotation

            # 🛑 stop 20 cm before block
            pose.pose.position.x -= 0.20

            self.get_logger().info(
                f"🎯 Sending Nav2 goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
            )

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            self.nav_client.wait_for_server()
            self.nav_client.send_goal_async(goal_msg)

            self.goal_sent = True

        except Exception:
            self.get_logger().info("⏳ Waiting for aruco TF...")

def main():
    rclpy.init()
    node = ArucoNavGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
