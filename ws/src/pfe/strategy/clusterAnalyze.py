
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from sklearn.cluster import DBSCAN
import numpy as np
import json
import time


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
        self.last_seen = time.time()

    def update(self, x, y, z, angle_z):
        """Met à jour la position de la caisse avec un lissage exponentiel."""
        alpha = 0.3
        self.x = self.x * (1 - alpha) + x * alpha
        self.y = self.y * (1 - alpha) + y * alpha
        self.z = self.z * (1 - alpha) + z * alpha
        self.angle_z = self.angle_z * (1 - alpha) + angle_z * alpha
        self.last_seen = time.time()

    def to_dict(self):
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "angle_z": self.angle_z,
            "color": self.color
        }


# ===========================================================
# CLASSE PRINCIPALE : Vision + Clustering
# ===========================================================
class ClusterAnalyzeNode(Node):

    def __init__(self):
        super().__init__('cluster_node')
        
        # --- ROS2 setup ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'topic_camera_image', self.callback, 20)
        self.pub_cluster = self.create_publisher(String, 'cluster_info', 10)

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
        self.memory = {}  # marker_id -> CaisseNoisette
        self.memory_timeout = 1.0  # secondes
        self.robot_pos = (0.0, 0.0)

        self.get_logger().info("✅ Vision + Clustering node started.")

    # =======================================================
    # CALLBACK PRINCIPAL
    # =======================================================
    def callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg)
        current_time = time.time()

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)

        # --- Détection ArUco ---
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                if int(marker_id) not in self.id_color_map:
                    continue  # ignorer les marqueurs non pertinents

                color = self.id_color_map[int(marker_id)]
                rvec, tvec = rvecs[i][0], tvecs[i][0]
                angle_z = np.degrees(rvec[2])

                # Lissage des positions dans la mémoire courte
                if marker_id in self.memory:
                    self.memory[marker_id].update(tvec[0], tvec[1], tvec[2], angle_z)
                else:
                    self.memory[marker_id] = CaisseNoisette(
                        id=int(marker_id),
                        x=float(tvec[0]),
                        y=float(tvec[1]),
                        z=float(tvec[2]),
                        angle_z=float(angle_z),
                        color=color
                    )

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        # --- Nettoyage mémoire ---
        to_delete = [mid for mid, c in self.memory.items() if current_time - c.last_seen > self.memory_timeout]
        for mid in to_delete:
            del self.memory[mid]

        # --- Liste des blocs actifs ---
        blocks = [c.to_dict() for c in self.memory.values()]

        if len(blocks) >= 2:
            # === CLUSTERING DBSCAN ===
            positions = np.array([[b["x"], b["y"]] for b in blocks])
            db = DBSCAN(eps=0.15, min_samples=2).fit(positions)
            labels = db.labels_

            clusters = []
            for label in set(labels):
                if label == -1:
                    continue
                cluster_points = [blocks[i] for i in range(len(labels)) if labels[i] == label]
                center = np.mean([[p["x"], p["y"]] for p in cluster_points], axis=0)
                size = len(cluster_points)
                clusters.append({'id': label, 'center': center.tolist(), 'size': size})

            # --- Calcul de priorité ---
            for c in clusters:
                dist = np.linalg.norm(np.array(c['center']) - np.array(self.robot_pos))
                c['score'] = (c['size'] * 3) + (1 / (dist + 0.01)) * 2

            if clusters:
                clusters.sort(key=lambda x: x['score'], reverse=True)
                best = clusters[0]

                # Publier le meilleur cluster
                msg_out = String()
                msg_out.data = json.dumps(best)
                self.pub_cluster.publish(msg_out)

                # Affichage visuel du cluster prioritaire
                cx, cy = int(best['center'][0]*1000 + 320), int(best['center'][1]*1000 + 240)
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                self.get_logger().info(
                    f"🟢 Cluster #{best['id']} - {best['size']} blocs | Score={best['score']:.2f} | Pos={best['center']}"
                )
        else:
            self.get_logger().info("Pas assez de blocs pour un regroupement.")

        # --- Affichage vidéo ---
        cv2.imshow("Camera Vision + Cluster", frame)
        cv2.waitKey(1)


# ===========================================================
# MAIN
# ===========================================================
def main(args=None):
    rclpy.init(args=args)
    node = ClusterAnalyzeNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
