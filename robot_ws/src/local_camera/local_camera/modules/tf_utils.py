import math
import tf_transformations
from geometry_msgs.msg import TransformStamped

class TfPublisher:
    def __init__(self, tf_broadcaster, get_clock):
        self.tf_broadcaster = tf_broadcaster
        self.get_clock = get_clock

    def publish_block_transforms(self, block_targets):
        now_msg = self.get_clock().now().to_msg()
        for cup_name, b in block_targets.items():
            if b is None:
                continue
            slot_idx = cup_name.split("_")[1]
            frame_name = f"block_target_{slot_idx}"
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = "base_link"
            t.child_frame_id = frame_name
            t.transform.translation.x = float(b.x)
            t.transform.translation.y = float(b.y)
            t.transform.translation.z = 0.0
            q = tf_transformations.quaternion_from_euler(0.0, 0.0, math.radians(b.yaw_deg))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

    def publish_pickup_target_tf(self, dx, dy, yaw_rad):
        """Publish a TF frame showing where base_link needs to move to."""
        now_msg = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = "base_link"
        t.child_frame_id = "pickup_target"
        t.transform.translation.x = float(dx)
        t.transform.translation.y = float(dy)
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
