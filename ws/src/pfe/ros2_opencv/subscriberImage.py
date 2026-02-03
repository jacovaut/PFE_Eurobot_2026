import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import json
import time
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations


# ===========================================================
# CLASSE : CaisseNoisette
# ===========================================================
class CaisseNoisette:
    """Classe représentant une caisse de noisettes détectée par la caméra."""

    def __init__(self, id, x, y, z, angle_z, color):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.angle_z = angle_z
        self.color = color
        self.last_seen = time.time()  # timestamp de dernière détection
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "aruco_pose", 10)

    def update(self, x, y, z, angle_z):
        """Met à jour la position de la caisse avec un lissage exponentiel."""
        alpha = 0.3  # facteur de lissage
        self.x = self.x * (1 - alpha) + x * alpha
        self.y = self.y * (1 - alpha) + y * alpha
        self.z = self.z * (1 - alpha) + z * alpha
        self.angle_z = self.angle_z * (1 - alpha) + angle_z * alpha
        self.last_seen = time.time()

    def to_dict(self):
        """Retourne un dictionnaire pour publication JSON."""
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "angle_z": self.angle_z,
            "color": self.color
        }

    def __repr__(self):
        return (f"CaisseNoisette({self.color}, "
                f"x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"angle_z={self.angle_z:.1f}°)")


# ===========================================================
# CLASSE : SubscriberNodeClass (avec mémoire courte)
# ===========================================================
class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        
        # --- ROS2 setup ---
        self.bridgeObject = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'topic_camera_image', self.listener_callbackFunction, 20)
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)

        # --- ArUco setup ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # --- Camera calibration ---
        self.camera_matrix = np.array([[600.0, 0.0, 320.0],
                                       [0.0, 600.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))
        self.marker_length = 0.05  # m

        # --- Map ID -> couleur ---
        self.id_color_map = {47: "jaune", 36: "bleu"}

        # --- Mémoire courte ---
        self.memory = {}  # dict: marker_id -> CaisseNoisette
        self.memory_timeout = 1.0  # secondes avant d'oublier un bloc

        self.get_logger().info('✅ Subscriber node with ArUco tracking + publishing started.')

    def listener_callbackFunction(self, imageMessage):
        frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        current_time = time.time()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                if int(marker_id) not in self.id_color_map:
                    continue  # ignorer marqueurs non pertinents

                color = self.id_color_map[int(marker_id)]
                rvec, tvec = rvecs[i][0], tvecs[i][0]
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

                # Si déjà en mémoire → mise à jour avec lissage
                if marker_id in self.memory:
                    self.memory[marker_id].update(float(tvec[0]), float(tvec[1]), float(tvec[2]), float(angle_z))
                else:
                    # Nouveau bloc détecté
                    self.memory[marker_id] = CaisseNoisette(
                        id=int(marker_id),
                        x=float(tvec[0]),
                        y=float(tvec[1]),
                        z=float(tvec[2]),
                        angle_z=float(angle_z),
                        color=color
                    )

                # Dessin des axes
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        # --- Nettoyage mémoire (blocs disparus depuis trop longtemps) ---
        to_delete = []
        for id_, caisse in self.memory.items():
            if current_time - caisse.last_seen > self.memory_timeout:
                to_delete.append(id_)
        for id_ in to_delete:
            del self.memory[id_]

        # --- Publication ROS2 ---
        active_caisses = [c.to_dict() for c in self.memory.values()]
        msg = String()
        msg.data = json.dumps(active_caisses)
        self.pub_blocks.publish(msg)

        # --- Logging ---
        if active_caisses:
            self.get_logger().info(f"📦 {len(active_caisses)} bloc(s) actif(s):")
            for c in active_caisses:
                self.get_logger().info(f" → {c}")
        else:
            self.get_logger().info("Aucun bloc actif détecté.")

        # --- Affichage vidéo ---
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
