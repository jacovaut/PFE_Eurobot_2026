import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sklearn.cluster import DBSCAN
import numpy as np
import cv2
import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class BlockDetection:
    marker_id: int
    color: str
    x: float
    y: float
    z: float
    angle_z_deg: float


class ClusterAnalyzeNode(Node):

    @staticmethod
    def _make_rect_zone(nom, centre_x_m, centre_y_m, taille_x_m, taille_y_m):
        demi_x = taille_x_m / 2.0
        demi_y = taille_y_m / 2.0
        return {
            "nom": nom,
            "centre": {"x": centre_x_m, "y": centre_y_m},
            "taille": {"x": taille_x_m, "y": taille_y_m},
            "bornes": {
                "x_min": centre_x_m - demi_x,
                "x_max": centre_x_m + demi_x,
                "y_min": centre_y_m - demi_y,
                "y_max": centre_y_m + demi_y,
            },
        }

    @staticmethod
    def point_dans_zone(x_m, y_m, zone):
        b = zone["bornes"]
        return b["x_min"] <= x_m <= b["x_max"] and b["y_min"] <= y_m <= b["y_max"]

    def __init__(self):
        super().__init__("cluster_node")

        # --- ROS2 setup ---
        self.subscription = self.create_subscription(
            String, "detected_blocks", self.callback, 20
        )
        self.pub_cluster = self.create_publisher(String, "cluster_info", 10)

        # --- Parameters (tunable later) ---
        self.team_color = self.declare_parameter("team_color", "jaune").value
        self.robot_marker_id = int(self.declare_parameter("robot_marker_id", 7).value)

        self.cluster_eps_m = float(self.declare_parameter("cluster_eps_m", 0.15).value)
        self.cluster_min_samples = int(self.declare_parameter("cluster_min_samples", 2).value)

        self.weight_color = float(self.declare_parameter("weight_color", 4.0).value)
        self.weight_distance = float(self.declare_parameter("weight_distance", 2.0).value)
        self.weight_alignment = float(self.declare_parameter("weight_alignment", 2.0).value)
        self.weight_size = float(self.declare_parameter("weight_size", 1.0).value)

        self.ally_color_penalty = float(
            self.declare_parameter("ally_color_penalty", 0.40).value
        )
        self.alignment_ref_deg = float(
            self.declare_parameter("alignment_ref_deg", 35.0).value
        )
        self.show_debug_window = bool(
            self.declare_parameter("show_debug_window", True).value
        )

        # Marker IDs for blocks
        self.id_color_map = {47: "jaune", 36: "bleu"}

        # Keep zone definitions available for next strategy steps
        self.zones_nid = {
            "nid_jaune": self._make_rect_zone("nid_jaune", 0.300, 1.775, 0.600, 0.450),
            "nid_bleu": self._make_rect_zone("nid_bleu", 2.700, 1.775, 0.600, 0.450),
        }
        self.zones_garde_manger = {
            "garde_manger_1": self._make_rect_zone("garde_manger_1", 0.700, 0.100, 0.200, 0.200),
            "garde_manger_2": self._make_rect_zone("garde_manger_2", 1.500, 0.100, 0.200, 0.200),
            "garde_manger_3": self._make_rect_zone("garde_manger_3", 2.300, 0.100, 0.200, 0.200),
            "garde_manger_4": self._make_rect_zone("garde_manger_4", 2.200, 0.800, 0.200, 0.200),
            "garde_manger_5": self._make_rect_zone("garde_manger_5", 1.500, 0.800, 0.200, 0.200),
            "garde_manger_6": self._make_rect_zone("garde_manger_6", 0.800, 0.800, 0.200, 0.200),
            "garde_manger_7": self._make_rect_zone("garde_manger_7", 1.750, 1.450, 0.200, 0.200),
            "garde_manger_8": self._make_rect_zone("garde_manger_8", 1.250, 1.450, 0.200, 0.200),
        }

        self.last_robot_pos: Optional[Tuple[float, float]] = None
        self.frame_count = 0

        # Debug map view config (table in meters)
        self.table_width_m = 3.0
        self.table_height_m = 2.0
        self.debug_canvas_w = 1200
        self.debug_canvas_h = 800
        self.debug_margin_px = 40
        self.debug_window_name = "Cluster Analyze Debug"
        self.debug_window_failed = False

        if self.show_debug_window:
            cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("✅ ClusterAnalyze node started.")
        self.get_logger().info(
            "Pipeline: 1) clusterisation DBSCAN, 2) selection meilleur cluster"
        )

    def _enemy_color(self) -> str:
        if self.team_color.lower() == "bleu":
            return "jaune"
        return "bleu"

    def _parse_detected_blocks(self, json_data: str) -> List[BlockDetection]:
        """Parse detected blocks from JSON string."""
        blocks: List[BlockDetection] = []
        try:
            data = json.loads(json_data)
            if not isinstance(data, list):
                return blocks
            
            for item in data:
                marker_id = int(item.get("marker_id", item.get("id", -1)))
                color = item.get("color", "")
                x = float(item.get("x", item.get("x_cam", 0.0)))
                y = float(item.get("y", item.get("y_cam", 0.0)))
                z = float(item.get("z", item.get("z_cam", 0.0)))
                angle_z_deg = float(item.get("angle_z_deg", math.degrees(float(item.get("yaw", 0.0)))))
                
                # Track robot position
                if marker_id == self.robot_marker_id:
                    self.last_robot_pos = (x, y)
                
                # Only add colored blocks (not robot)
                if marker_id in self.id_color_map:
                    blocks.append(
                        BlockDetection(
                            marker_id=marker_id,
                            color=color,
                            x=x,
                            y=y,
                            z=z,
                            angle_z_deg=angle_z_deg,
                        )
                    )
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"Failed to parse detected_blocks JSON: {e}")
        
        return blocks

    def _build_clusters(self, blocks: List[BlockDetection]) -> List[Dict]:
        if len(blocks) < self.cluster_min_samples:
            return []

        points = np.array([[b.x, b.y] for b in blocks])
        labels = DBSCAN(eps=self.cluster_eps_m, min_samples=self.cluster_min_samples).fit_predict(points)

        clusters = []
        for label in sorted(set(labels)):
            if label == -1:
                continue

            members = [blocks[i] for i in range(len(blocks)) if labels[i] == label]
            center_x = float(np.mean([m.x for m in members]))
            center_y = float(np.mean([m.y for m in members]))
            angle_std_deg = float(np.std([m.angle_z_deg for m in members])) if len(members) > 1 else 0.0

            count_jaune = sum(1 for m in members if m.color == "jaune")
            count_bleu = sum(1 for m in members if m.color == "bleu")

            clusters.append(
                {
                    "id": int(label),
                    "members": members,
                    "size": len(members),
                    "center": [center_x, center_y],
                    "count_jaune": int(count_jaune),
                    "count_bleu": int(count_bleu),
                    "angle_std_deg": angle_std_deg,
                }
            )

        return clusters

    def _score_cluster(self, cluster: Dict) -> Dict:
        enemy_color = self._enemy_color()

        size = max(cluster["size"], 1)
        enemy_count = sum(1 for m in cluster["members"] if m.color == enemy_color)
        ally_count = size - enemy_count

        color_score = (enemy_count - self.ally_color_penalty * ally_count) / float(size)

        if self.last_robot_pos is None:
            dist_m = 999.0
            distance_score = 0.0
        else:
            dx = cluster["center"][0] - self.last_robot_pos[0]
            dy = cluster["center"][1] - self.last_robot_pos[1]
            dist_m = math.hypot(dx, dy)
            distance_score = 1.0 / (dist_m + 0.05)

        align_score = max(0.0, 1.0 - (cluster["angle_std_deg"] / max(self.alignment_ref_deg, 1e-6)))
        size_score = min(float(size), 8.0) / 8.0

        total_score = (
            self.weight_color * color_score
            + self.weight_distance * distance_score
            + self.weight_alignment * align_score
            + self.weight_size * size_score
        )

        return {
            "id": cluster["id"],
            "size": cluster["size"],
            "center": cluster["center"],
            "count_jaune": cluster["count_jaune"],
            "count_bleu": cluster["count_bleu"],
            "distance_m": float(dist_m),
            "angle_std_deg": float(cluster["angle_std_deg"]),
            "color_score": float(color_score),
            "distance_score": float(distance_score),
            "align_score": float(align_score),
            "size_score": float(size_score),
            "score": float(total_score),
            "member_ids": [int(m.marker_id) for m in cluster["members"]],
            "member_colors": [m.color for m in cluster["members"]],
        }

    def _table_to_px(self, x_m: float, y_m: float) -> Tuple[int, int]:
        usable_w = self.debug_canvas_w - 2 * self.debug_margin_px
        usable_h = self.debug_canvas_h - 2 * self.debug_margin_px

        x_clamped = max(0.0, min(self.table_width_m, x_m))
        y_clamped = max(0.0, min(self.table_height_m, y_m))

        px = int(self.debug_margin_px + (x_clamped / self.table_width_m) * usable_w)
        py = int(self.debug_canvas_h - self.debug_margin_px - (y_clamped / self.table_height_m) * usable_h)
        return px, py

    def _draw_debug_window(self, blocks: List[BlockDetection], scored_clusters: List[Dict], best_cluster: Optional[Dict]) -> None:
        if not self.show_debug_window or self.debug_window_failed:
            return

        canvas = np.full((self.debug_canvas_h, self.debug_canvas_w, 3), 245, dtype=np.uint8)

        # Table border
        tl = (self.debug_margin_px, self.debug_margin_px)
        br = (self.debug_canvas_w - self.debug_margin_px, self.debug_canvas_h - self.debug_margin_px)
        cv2.rectangle(canvas, tl, br, (30, 30, 30), 2)

        # Grid every 0.5m
        for gx in np.arange(0.5, self.table_width_m, 0.5):
            x1, y1 = self._table_to_px(gx, 0.0)
            x2, y2 = self._table_to_px(gx, self.table_height_m)
            cv2.line(canvas, (x1, y1), (x2, y2), (220, 220, 220), 1)

        for gy in np.arange(0.5, self.table_height_m, 0.5):
            x1, y1 = self._table_to_px(0.0, gy)
            x2, y2 = self._table_to_px(self.table_width_m, gy)
            cv2.line(canvas, (x1, y1), (x2, y2), (220, 220, 220), 1)

        # Zones: nids and garde-manger
        for zone in self.zones_nid.values():
            b = zone["bornes"]
            x_min, y_min = self._table_to_px(b["x_min"], b["y_min"])
            x_max, y_max = self._table_to_px(b["x_max"], b["y_max"])
            p1 = (min(x_min, x_max), min(y_min, y_max))
            p2 = (max(x_min, x_max), max(y_min, y_max))

            cv2.rectangle(canvas, p1, p2, (170, 90, 20), 2)
            label = zone["nom"].replace("nid_", "N:")
            cv2.putText(
                canvas,
                label,
                (p1[0] + 4, p1[1] + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (170, 90, 20),
                1,
                cv2.LINE_AA,
            )

        for zone in self.zones_garde_manger.values():
            b = zone["bornes"]
            x_min, y_min = self._table_to_px(b["x_min"], b["y_min"])
            x_max, y_max = self._table_to_px(b["x_max"], b["y_max"])
            p1 = (min(x_min, x_max), min(y_min, y_max))
            p2 = (max(x_min, x_max), max(y_min, y_max))

            cv2.rectangle(canvas, p1, p2, (120, 120, 120), 1)
            gm_name = zone["nom"].replace("garde_manger_", "GM")
            cv2.putText(
                canvas,
                gm_name,
                (p1[0] + 2, p1[1] + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (100, 100, 100),
                1,
                cv2.LINE_AA,
            )

        # Blocks
        for b in blocks:
            px, py = self._table_to_px(b.x, b.y)
            color_bgr = (0, 180, 255) if b.color == "jaune" else (255, 100, 0)
            cv2.circle(canvas, (px, py), 7, color_bgr, -1)
            cv2.putText(
                canvas,
                f"{b.marker_id}",
                (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (40, 40, 40),
                1,
                cv2.LINE_AA,
            )

        # Robot pose if available
        if self.last_robot_pos is not None:
            rx, ry = self._table_to_px(self.last_robot_pos[0], self.last_robot_pos[1])
            cv2.circle(canvas, (rx, ry), 8, (0, 0, 0), 2)
            cv2.putText(
                canvas,
                "robot",
                (rx + 10, ry + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )

        # Clusters
        best_id = None if best_cluster is None else best_cluster.get("id")
        for c in scored_clusters:
            cx, cy = self._table_to_px(c["center"][0], c["center"][1])
            is_best = (best_id is not None and c["id"] == best_id)
            ring_color = (0, 0, 255) if is_best else (80, 80, 80)
            thickness = 3 if is_best else 2
            cv2.circle(canvas, (cx, cy), 18, ring_color, thickness)
            cv2.putText(
                canvas,
                f"C{c['id']} s={c['score']:.2f}",
                (cx + 20, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                ring_color,
                1,
                cv2.LINE_AA,
            )

        # Header text
        header = f"blocks={len(blocks)} clusters={len(scored_clusters)} team={self.team_color}"
        cv2.putText(canvas, header, (20, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 30, 30), 2, cv2.LINE_AA)

        if best_cluster is not None:
            best_txt = (
                f"best=C{best_cluster['id']} score={best_cluster['score']:.2f} "
                f"dist={best_cluster['distance_m']:.2f}m size={best_cluster['size']}"
            )
        else:
            best_txt = "best=none"

        cv2.putText(canvas, best_txt, (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)

        try:
            cv2.imshow(self.debug_window_name, canvas)
            cv2.waitKey(1)
        except cv2.error as e:
            self.debug_window_failed = True
            self.get_logger().warning(f"Cluster debug window disabled: {e}")

    def callback(self, msg):
        self.frame_count += 1
        
        # Parse detected blocks from JSON
        blocks = self._parse_detected_blocks(msg.data)
        clusters = self._build_clusters(blocks)

        scored_clusters = [self._score_cluster(c) for c in clusters]
        scored_clusters.sort(key=lambda c: c["score"], reverse=True)
        best_cluster = scored_clusters[0] if scored_clusters else None

        msg_out = String()
        msg_out.data = json.dumps(
            {
                "step_1_clusterisation": {
                    "num_blocks": len(blocks),
                    "num_clusters": len(scored_clusters),
                },
                "step_2_selection": {
                    "team_color": self.team_color,
                    "enemy_color": self._enemy_color(),
                    "robot_pose_camera_xy": list(self.last_robot_pos) if self.last_robot_pos else None,
                    "best_cluster": best_cluster,
                },
                "clusters": scored_clusters,
            }
        )
        self.pub_cluster.publish(msg_out)

        self._draw_debug_window(blocks, scored_clusters, best_cluster)

        if self.frame_count % 10 == 0:
            if best_cluster is None:
                self.get_logger().info(
                    f"Aucun cluster exploitable | detections={len(blocks)}"
                )
            else:
                self.get_logger().info(
                    "Best cluster #{} | size={} | score={:.3f} | dist={:.3f} m | "
                    "jaune={} bleu={} | align_std={:.1f} deg".format(
                        best_cluster["id"],
                        best_cluster["size"],
                        best_cluster["score"],
                        best_cluster["distance_m"],
                        best_cluster["count_jaune"],
                        best_cluster["count_bleu"],
                        best_cluster["angle_std_deg"],
                    )
                )

    def destroy_node(self):
        if self.show_debug_window and not self.debug_window_failed:
            try:
                cv2.destroyWindow(self.debug_window_name)
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClusterAnalyzeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
