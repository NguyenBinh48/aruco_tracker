# Copyright 2023 Josh Newans, modified for ArUco tracking
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np

class DetectAruco(Node):

    def __init__(self):
        super().__init__('detect_aruco')

        self.get_logger().info('Looking for ArUco marker...')
        # Subscriptions and Publishers
        self.image_sub = self.create_subscription(
            Image,
            '/image_in',
            self.callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        self.image_out_pub = self.create_publisher(Image, '/image_out', 1)
        self.aruco_pos_pub = self.create_publisher(Float32MultiArray, '/aruco_position', 10)
        self.ball_pub = self.create_publisher(Point, '/detected_aruco', 1)  # For backward compatibility


        # Camera calibration (Example values, replace with real ones)
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)
        self.marker_id = 23  
        self.marker_size = 0.05  # meters
        self.publish_images = True

        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

        # CvBridge
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Convert to grayscale for detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        point_out = Point()
        pos_msg = Float32MultiArray()

        if ids is not None and self.marker_id in ids.flatten():
            # Find the target marker
            index = np.where(ids.flatten() == self.marker_id)[0][0]
            # Estimate pose
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[index], self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            if tvec is not None:
                # Convert position to cm
                # x, y, z = tvec[0][0] * 100, tvec[0][1] * 100, tvec[0][2] * 100
                x, y, z = tvec[0][0] * 100  # Convert meters to cm

                pos_msg.data = [x, y, z]
                self.aruco_pos_pub.publish(pos_msg)

                # For compatibility with original ball tracker
                point_out.x = x
                point_out.y = y
                point_out.z = z
                self.ball_pub.publish(point_out)

                self.get_logger().info(f'Detected ArUco ID {self.marker_id}: x={x:.1f} cm, y={y:.1f} cm, z={z:.1f} cm')

                if self.publish_images:
                    # Visualize marker
                    aruco.drawDetectedMarkers(cv_image, corners)
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size)
                    # Publish visualized image
                    try:
                        img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                        img_msg.header = data.header
                        self.image_out_pub.publish(img_msg)
                    except CvBridgeError as e:
                        self.get_logger().error(f'CvBridge Error publishing image: {e}')

        else:
            self.get_logger().debug('No target ArUco marker detected')

def main(args=None):
    rclpy.init(args=args)
    detect_aruco = DetectAruco()
    rclpy.spin(detect_aruco)
    detect_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()