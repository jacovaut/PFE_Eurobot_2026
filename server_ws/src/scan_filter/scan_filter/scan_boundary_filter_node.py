"""
scan_boundary_filter_node.py
----------------------------
Subscribes to a PointCloud2 produced by the lidar (in laser_frame) and drops
every point whose map-frame coordinates fall outside the game table boundaries.
The filtered cloud is republished on a new topic with the **same frame_id** as
the input so that Nav2's costmap can still apply the laser_frame → map TF
internally — we are only throwing away points, not re-projecting them.

Parameters
----------
input_topic       : str   default '/scan_cloud'
output_topic      : str   default '/scan_cloud_filtered'
map_frame         : str   default 'map'
map_x_min         : float default  0.0   (m, map frame)
map_x_max         : float default  3.0   (m, map frame)
map_y_min         : float default  0.0   (m, map frame)
map_y_max         : float default  2.0   (m, map frame)
margin            : float default  0.05  (m, shrinks all four sides equally)
tf_timeout_sec    : float default  0.05  (s, max wait for TF lookup)
"""

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import tf2_ros


class ScanBoundaryFilterNode(Node):
    def __init__(self):
        super().__init__('scan_boundary_filter_node')

        # --- parameters ---------------------------------------------------
        self.declare_parameter('input_topic',    '/scan_cloud')
        self.declare_parameter('output_topic',   '/scan_cloud_filtered')
        self.declare_parameter('map_frame',      'map')
        self.declare_parameter('map_x_min',       0.0)
        self.declare_parameter('map_x_max',       3.0)
        self.declare_parameter('map_y_min',       0.0)
        self.declare_parameter('map_y_max',       2.0)
        self.declare_parameter('margin',          0.05)
        self.declare_parameter('tf_timeout_sec',  0.05)

        input_topic    = self.get_parameter('input_topic').value
        output_topic   = self.get_parameter('output_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        margin         = self.get_parameter('margin').value
        self.tf_timeout = Duration(
            seconds=int(self.get_parameter('tf_timeout_sec').value),
            nanoseconds=int(
                (self.get_parameter('tf_timeout_sec').value % 1) * 1e9
            ),
        )

        # Shrink the allowed region by the margin on all four sides
        self.x_min = self.get_parameter('map_x_min').value + margin
        self.x_max = self.get_parameter('map_x_max').value - margin
        self.y_min = self.get_parameter('map_y_min').value + margin
        self.y_max = self.get_parameter('map_y_max').value - margin

        # --- TF2 ----------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- pub / sub ----------------------------------------------------
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, reliable_qos)

        # The lidar driver uses BEST_EFFORT; use sensor_data QoS to match.
        self.create_subscription(
            PointCloud2,
            input_topic,
            self._cloud_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'scan_boundary_filter: {input_topic} → {output_topic} | '
            f'x=[{self.x_min:.3f}, {self.x_max:.3f}] '
            f'y=[{self.y_min:.3f}, {self.y_max:.3f}] (map frame, margin={margin} m)'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        """Return the 4×4 homogeneous matrix for a geometry_msgs Transform."""
        t = transform.transform.translation
        r = transform.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        # fmt: off
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w),   t.x],
            [  2*(x*y + z*w),   1 - 2*(x*x + z*z),   2*(y*z - x*w),   t.y],
            [  2*(x*z - y*w),     2*(y*z + x*w),   1 - 2*(x*x + y*y), t.z],
            [0,                   0,                   0,               1.0],
        ], dtype=np.float64)
        # fmt: on

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def _cloud_cb(self, msg: PointCloud2) -> None:
        n_points = msg.width * msg.height
        if n_points == 0:
            self.publisher.publish(msg)
            return

        # --- 1. TF lookup -----------------------------------------------
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=self.tf_timeout,
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warning(
                f'TF lookup failed ({msg.header.frame_id} → {self.map_frame}): {exc}',
                throttle_duration_sec=1.0,
            )
            return

        mat = self._transform_to_matrix(tf_stamped)

        # --- 2. Decode XYZ from PointCloud2 (float32 XYZ at offset 0) ---
        # scan_pointcloud_node packs points as struct.pack('fff', x, y, z)
        # so point_step=12 and fields are x@0, y@4, z@8 — all float32.
        raw_bytes = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)
        xyz_local = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, msg.point_step // 4)[:, :3].astype(np.float64)

        # --- 3. Transform to map frame (batched) -------------------------
        ones = np.ones((n_points, 1), dtype=np.float64)
        pts_map = (mat @ np.hstack([xyz_local, ones]).T).T  # (N, 4)

        # --- 4. Filter by map bounds -------------------------------------
        mask = (
            (pts_map[:, 0] >= self.x_min) & (pts_map[:, 0] <= self.x_max) &
            (pts_map[:, 1] >= self.y_min) & (pts_map[:, 1] <= self.y_max)
        )

        # --- 5. Rebuild PointCloud2 (original laser_frame coords) --------
        filtered = raw_bytes[mask]

        out = PointCloud2()
        out.header     = msg.header          # keep original frame_id — Nav2 handles TF
        out.height     = 1
        out.width      = len(filtered)
        out.fields     = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step   = msg.point_step * len(filtered)
        out.is_dense   = True
        out.data       = filtered.tobytes()

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBoundaryFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
