import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import json


# ===========================================================
# CLASSE : CaisseNoisette
# ===========================================================
class CaisseNoisette:
    """Classe représentant une caisse de noisettes détectée par la caméra."""

    compteur_jaune = 0
    compteur_bleu = 0

    def __init__(self, id, x, y, z, angle_z, color):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.angle_z = angle_z
        self.color = color

        # Création d’un identifiant unique (même si les ID ArUco se répètent)
        if color == "jaune":
            CaisseNoisette.compteur_jaune += 1
            self.index = CaisseNoisette.compteur_jaune
        elif color == "bleu":
            CaisseNoisette.compteur_bleu += 1
            self.index = CaisseNoisette.compteur_bleu
        else:
            self.index = 0

    def nom(self):
        """Nom unique combinant la couleur et un index."""
        return f"{self.color}_{self.index}"

    def to_dict(self):
        """Retourne un dictionnaire compatible avec JSON."""
        return {
            "id": self.id,
            "name": self.nom(),
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "angle_z": self.angle_z,
            "color": self.color
        }

    def __repr__(self):
        return (f"CaisseNoisette({self.nom()}, "
                f"x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"angle_z={self.angle_z:.1f}°)")


# ===========================================================
# CLASSE : SubscriberNodeClass (ROS2)
# ===========================================================
class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        
        # --- ROS2 setup ---
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20

        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction,
            self.queueSize
        )

        # Publisher vers le topic /detected_blocks
        self.pub_blocks = self.create_publisher(String, 'detected_blocks', 10)

        # --- ArUco setup ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # --- Calibration caméra ---
        self.camera_matrix = np.array([[600.0, 0.0, 320.0],
                                       [0.0, 600.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))
        self.marker_length = 0.05  # m

        # --- Map ID -> couleur ---
        self.id_color_map = {
            47: "jaune",
            36: "bleu"
        }

        # Liste locale de caisses
        self.caisses = []

        self.get_logger().info('✅ Subscriber node with ArUco detection + publishing started.')

    def listener_callbackFunction(self, imageMessage):
        frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        self.caisses.clear()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = rvecs[i][0], tvecs[i][0]
                angle_z = np.degrees(rvec[2])
                color = self.id_color_map.get(int(marker_id), "inconnu")

                caisse = CaisseNoisette(
                    id=int(marker_id),
                    x=float(tvec[0]),
                    y=float(tvec[1]),
                    z=float(tvec[2]),
                    angle_z=float(angle_z),
                    color=color
                )
                self.caisses.append(caisse)

                # Afficher les axes de chaque marqueur
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

            # Afficher les logs dans le terminal
            for caisse in self.caisses:
                self.get_logger().info(str(caisse))

            # --- Publier les données JSON ---
            msg = String()
            msg.data = json.dumps([c.to_dict() for c in self.caisses])
            self.pub_blocks.publish(msg)

        # Afficher la vidéo
        cv2.imshow("Camera Video with ArUco Pose", frame)
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
