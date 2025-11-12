
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json
from sklearn.cluster import DBSCAN

class ClusterAnalyzerNode(Node):
    def __init__(self):
        super().__init__('cluster_node')

        self.sub_blocks = self.create_subscription(
            String, 'detected_blocks', self.blocks_callback, 10
        )

        self.pub_cluster_info = self.create_publisher(String, 'cluster_info', 10)
        self.robot_pos = (0.0, 0.0)
        self.get_logger().info("✅ ClusterAnalyzerNode started (DBSCAN clustering).")

    def blocks_callback(self, msg):
        try:
            blocks = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON parsing error: {e}")
            return

        if len(blocks) < 2:
            self.get_logger().info("Pas assez de blocs détectés pour le regroupement.")
            return

        # --- DBSCAN Clustering ---
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

        # --- Priorisation ---
        for c in clusters:
            dist = np.linalg.norm(np.array(c['center']) - np.array(self.robot_pos))
            c['score'] = (c['size'] * 3) + (1 / (dist + 0.01)) * 2

        if clusters:
            clusters.sort(key=lambda x: x['score'], reverse=True)
            best = clusters[0]
            self.get_logger().info(
                f"🟢 Cluster #{best['id']} - {best['size']} blocs | "
                f"Score={best['score']:.2f} | Position={best['center']}"
            )

            # Publier le cluster prioritaire
            msg_out = String()
            msg_out.data = json.dumps(best)
            self.pub_cluster_info.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ClusterAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
