import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge


class PublisherNodeClass(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # -------------------------
        # ROS params (tweak at runtime)
        # -------------------------
        self.declare_parameter("device", 0)
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("fps", 30.0)

        # Try these if your camera/driver supports them
        self.declare_parameter("autofocus", 0)   # 0=off, 1=on (often unsupported on Linux UVC via OpenCV)
        self.declare_parameter("focus", 0)       # manual focus value if supported
        self.declare_parameter("auto_exposure", 0)  # 0/1 depending on backend; may be ignored
        self.declare_parameter("exposure", -6.0)    # many cams use negative; totally driver dependent
        self.declare_parameter("gain", 0.0)
        self.declare_parameter("brightness", 0.0)
        self.declare_parameter("contrast", 0.0)
        self.declare_parameter("sharpness", 0.0)

        # Publishing
        self.bridge = CvBridge()
        self.topic_gray = "topic_camera_image"     # keep your existing topic
        self.publisher_gray = self.create_publisher(Image, self.topic_gray, 10)

        # If you want a debug color topic, uncomment:
        # self.topic_bgr = "topic_camera_bgr"
        # self.publisher_bgr = self.create_publisher(Image, self.topic_bgr, 10)

        # -------------------------
        # Camera init
        # -------------------------
        dev = int(self.get_parameter("device").value)
        self.camera = cv2.VideoCapture(dev, cv2.CAP_V4L2)  # V4L2 is best on Linux

        if not self.camera.isOpened():
            self.get_logger().error(f"Failed to open camera device {dev}")
            raise RuntimeError("Could not open camera")

        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)

        # Request format & resolution (driver may refuse and pick closest)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.camera.set(cv2.CAP_PROP_FPS, fps)

        # Try to reduce latency (may or may not be honored)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Apply optional tuning (best-effort)
        self._apply_camera_controls()

        # Log actual negotiated settings
        aw = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        ah = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        afps = self.camera.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera negotiated: {aw:.0f}x{ah:.0f} @ {afps:.1f} FPS")

        # Timer: match fps but don’t overrun
        period = 1.0 / max(1.0, fps)
        self.timer = self.create_timer(period, self.timer_cb)
        self.i = 0

    def _apply_camera_controls(self):
        # These are highly driver dependent; we just try.
        def set_prop(prop, value, name):
            if value is None:
                return
            ok = self.camera.set(prop, value)
            self.get_logger().info(f"Set {name}={value} -> {'OK' if ok else 'IGNORED'}")

        set_prop(cv2.CAP_PROP_AUTOFOCUS, float(self.get_parameter("autofocus").value), "autofocus")
        set_prop(cv2.CAP_PROP_FOCUS, float(self.get_parameter("focus").value), "focus")

        # Exposure: OpenCV uses different semantics per backend.
        # On V4L2, CAP_PROP_AUTO_EXPOSURE often expects:
        # 1 = manual, 3 = auto (but varies). We try your param as-is.
        set_prop(cv2.CAP_PROP_AUTO_EXPOSURE, float(self.get_parameter("auto_exposure").value), "auto_exposure")
        set_prop(cv2.CAP_PROP_EXPOSURE, float(self.get_parameter("exposure").value), "exposure")

        set_prop(cv2.CAP_PROP_GAIN, float(self.get_parameter("gain").value), "gain")
        set_prop(cv2.CAP_PROP_BRIGHTNESS, float(self.get_parameter("brightness").value), "brightness")
        set_prop(cv2.CAP_PROP_CONTRAST, float(self.get_parameter("contrast").value), "contrast")
        set_prop(cv2.CAP_PROP_SHARPNESS, float(self.get_parameter("sharpness").value), "sharpness")

    def timer_cb(self):
        success, frame = self.camera.read()
        if not success or frame is None:
            self.get_logger().warn("No frame captured from camera")
            return

        # DO NOT resize for long-range detection (keep native res)
        # If you *must* downscale, use INTER_AREA and do it in subscriber:
        # frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        msg_gray = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")
        self.publisher_gray.publish(msg_gray)

        # Optional debug publish:
        # msg_bgr = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # self.publisher_bgr.publish(msg_bgr)

        # Don’t spam logs at 30 FPS
        if self.i % 30 == 0:
            self.get_logger().info(f"Publishing frame {self.i}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNodeClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()