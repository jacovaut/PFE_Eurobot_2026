import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


class ScanPointCloudNode(Node):
    def __init__(self):
        super().__init__('scan_pointcloud_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pointcloud_topic', '/scan_cloud')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('z_offset', 0.0)

        scan_topic = str(self.get_parameter('scan_topic').value)
        pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)

        self.output_frame = str(self.get_parameter('output_frame').value)
        self.z_offset = float(self.get_parameter('z_offset').value)

        pointcloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(PointCloud2, pointcloud_topic, pointcloud_qos)
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Publishing point clouds on {pointcloud_topic} from scans on {scan_topic}'
        )

    def scan_callback(self, scan_msg: LaserScan) -> None:
        frame_id = self.output_frame or scan_msg.header.frame_id
        if not frame_id:
            self.get_logger().warning('Dropping scan with empty frame_id and no output_frame override')
            return

        points = bytearray()
        angle = scan_msg.angle_min
        for distance in scan_msg.ranges:
            if math.isfinite(distance) and distance >= scan_msg.range_min:
                if scan_msg.range_max <= 0.0 or distance <= scan_msg.range_max:
                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    points.extend(struct.pack('fff', x, y, self.z_offset))
            angle += scan_msg.angle_increment

        cloud_msg = PointCloud2()
        cloud_msg.header = scan_msg.header
        cloud_msg.header.frame_id = frame_id
        cloud_msg.height = 1
        cloud_msg.width = len(points) // 12
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = bytes(points)

        self.publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()