import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import json
import time
import tf_transformations
from tf2_ros import TransformBroadcaster

# ===========================================================
# CLASSE : CaisseNoisette
# ===========================================================
class CaisseNoisette:
    """Classe représentant une caisse de noisettes détectée par la caméra."""

    def __init__(self, id, index, x, y, z, angle_z, color):
        self.id = id
        self.index = index  # unique per instance
        self.x = x
        self.y = y
        self.z = z
        self.angle_z = angle_z
        self.color = color
        self.last_seen = time.time()

    def update(self, x, y, z, angle_z):
        alpha = 0.3
        self.x = self.x * (1 - alpha) + x * alpha
        self.y = self.y * (1 - alpha) + y * alpha
        self.z = self.z * (1 - alpha) + z * alpha
        self.angle_z = self.angle_z * (1 - alpha) + angle_z * alpha
        self.last_seen = time.time()

    def to_dict(self):
        return {
            "id": self.id,
            "index": self.index,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "angle_z": self.angle_z,
            "color": self.color
        }

    def __repr__(self):
        return (f"CaisseNoisette({self.color}, "
                f"id={self.id}, index={self.index}, "
                f"x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"angle_z={self.angle_z:.1f}°)")

# ===========================================================
# NODE
# ===========================================================
class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        
        # --- ROS2 setup ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'topic_camera_image', self.listener_callback, 20)
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- ArUco setup ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.05  # meters

        # --- Camera calibration ---
        self.camera_matrix = np.array([[600.0, 0.0, 320.0],
                                       [0.0, 600.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))

        # --- Map ID -> couleur ---
        self.id_color_map = {47: "jaune", 36: "bleu"}

        # --- Mémoire courte ---
        self.memory = {}  # key = (marker_id, index)
        self.memory_timeout = 1.0  # seconds

        self.get_logger().info('✅ Subscriber node with ArUco tracking + RViz markers started.')

    def listener_callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        current_time = time.time()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                if int(marker_id) not in self.id_color_map:
                    continue

                color = self.id_color_map[int(marker_id)]
                rvec, tvec = rvecs[i][0], tvecs[i][0]
                angle_z = np.degrees(rvec[2])

                key = (marker_id, i)  # unique per instance

                # --- Memory update ---
                if key in self.memory:
                    self.memory[key].update(float(tvec[0]), float(tvec[1]), float(tvec[2]), float(angle_z))
                else:
                    self.memory[key] = CaisseNoisette(
                        id=int(marker_id),
                        index=i,
                        x=float(tvec[0]),
                        y=float(tvec[1]),
                        z=float(tvec[2]),
                        angle_z=float(angle_z),
                        color=color
                    )

                # --- TF ---
                R, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = R
                quat = tf_transformations.quaternion_from_matrix(T)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_link"

                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])

                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                self.pose_pub.publish(pose_msg)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_link"
                t.child_frame_id = f"aruco_{marker_id}"

                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                self.tf_broadcaster.sendTransform(t)

                # --- Draw axes for camera debug ---
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        # --- Clean memory ---
        to_delete = [k for k, c in self.memory.items() if current_time - c.last_seen > self.memory_timeout]
        for k in to_delete:
            del self.memory[k]

        # --- MarkerArray for RViz ---
        marker_array = MarkerArray()
        for c in self.memory.values():
            marker = Marker()
            marker.header.frame_id = "arducam_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = c.id * 100 + c.index  # unique
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = c.x
            marker.pose.position.y = c.y
            marker.pose.position.z = c.z

            quat = tf_transformations.quaternion_from_euler(0, 0, c.angle_z)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            if c.color == "jaune":
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif c.color == "bleu":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 0.9

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        # --- JSON output ---
        msg = String()
        msg.data = json.dumps([c.to_dict() for c in self.memory.values()])
        self.pub_blocks.publish(msg)

        # --- Camera debug window ---
        cv2.imshow("Camera Video with Tracking", frame)
        cv2.waitKey(1)

# ===========================================================
# MAIN
# ===========================================================
def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNodeClass()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
