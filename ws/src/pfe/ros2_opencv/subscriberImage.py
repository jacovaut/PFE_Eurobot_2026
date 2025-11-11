import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np

class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20

        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction,
            self.queueSize
        )
        self.subscription

        # --- Initialize ArUco detector ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # --- Load camera calibration (example hardcoded values) ---
        # Replace with your own calibration from e.g. camera_info or YAML file
        self.camera_matrix = np.array([[600.0, 0.0, 320.0],
                                       [0.0, 600.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))  # if not using distortion
        self.marker_length = 0.05  # meters

        self.get_logger().info('Subscriber node with ArUco pose estimation started.')

    def listener_callbackFunction(self, imageMessage):
        self.get_logger().info('Image frame received')

        # Convert ROS Image message to OpenCV image
        frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # --- Detect ArUco markers ---
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # --- Estimate pose for each detected marker ---
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # Draw the 3D axis on top of the marker
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

                # Log position and orientation
                self.get_logger().info(
                    f"Marker ID {marker_id}: "
                    f"Position (x={tvec[0]:.3f} m, y={tvec[1]:.3f} m, z={tvec[2]:.3f} m)"
                )

        # Display result
        cv2.imshow("Camera Video with ArUco Pose", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    subscriberNode = SubscriberNodeClass()
    rclpy.spin(subscriberNode)
    subscriberNode.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


