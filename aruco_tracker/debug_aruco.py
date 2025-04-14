import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDebugViewer(Node):
    def __init__(self):
        super().__init__('debug_aruco')
        self.get_logger().info('Starting ArUco debug viewer...')

        # Subscribe to raw camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
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

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and self.marker_id in ids.flatten():
            # Find the target marker
            index = np.where(ids.flatten() == self.marker_id)[0][0]
            # Estimate pose
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[index], self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            if tvec is not None:
                # Draw marker outline and axes
                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.5)

                # Draw bounding box
                corner_points = corners[index][0].astype(int)
                top_left = tuple(corner_points[0])
                bottom_right = tuple(corner_points[2])
                cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

                # Display position
                x, y, z = tvec[0][0] * 100, tvec[0][1] * 100, tvec[0][2] * 100
                cv2.putText(
                    frame,
                    f'ID {self.marker_id}: x={x:.1f}, y={y:.1f}, z={z:.1f} cm',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )
                self.get_logger().info(f'Detected ArUco ID {self.marker_id}: x={x:.1f}, y={y:.1f}, z={z:.1f} cm')
        else:
            cv2.putText(
                frame,
                'No ArUco marker detected',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1
            )
            self.get_logger().debug('No target ArUco marker detected')

        # Display the image
        cv2.imshow('ArUco Debug Viewer', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDebugViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()