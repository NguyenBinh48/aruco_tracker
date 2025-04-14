import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDebug(Node):
    def __init__(self):
        super().__init__('debug')
        self.get_logger().info('Starting ArUco debug viewer...')

        # Subscribe to raw camera image
        self.subscription = self.create_subscription(
            Image,
            '/image_out',
            self.image_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # CvBridge for image conversion
        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('marker_id', 23)
        self.declare_parameter('marker_size', 0.05)  # meters
        self.declare_parameter('camera_matrix', [
            1000.0, 0.0, 320.0,
            0.0, 1000.0, 240.0,
            0.0, 0.0, 1.0
        ])
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])

        # Get parameters
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        camera_matrix_flat = self.get_parameter('camera_matrix').get_parameter_value().double_array_value
        self.camera_matrix = np.array(camera_matrix_flat, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(
            self.get_parameter('dist_coeffs').get_parameter_value().double_array_value,
            dtype=np.float32
        )

        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return



        # Display the image
        cv2.imshow('ArUco Debug Viewer', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()