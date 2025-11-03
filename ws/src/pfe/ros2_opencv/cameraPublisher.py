import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class PublisherNodeClass(Node):
    
    def __init__(self):
        super().__init__('publisher_node')
        
        # Camera setup
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        # ROS publisher setup
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        
        # Timer for publishing
        self.periodCommunication = 0.1  # 10 Hz
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        
        self.i = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        
        if not success:
            self.get_logger().warn("No frame captured from camera")
            return
        
        # Resize if needed
        frame = cv2.resize(frame, (680, 480), interpolation=cv2.INTER_CUBIC)
        
        # --- Convert to grayscale ---
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Convert to ROS Image message
        ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(gray_frame, encoding="mono8")
        self.publisher.publish(ROS2ImageMessage)
        
        self.get_logger().info(f'Publishing grayscale image number {self.i}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PublisherNodeClass()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

