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
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class FollowAruco(Node):

    def __init__(self):
        super().__init__('follow_aruco')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/aruco_position',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Declare parameters
        self.declare_parameter('rcv_timeout_secs', 1.0)
        self.declare_parameter('angular_chase_multiplier', 0.01)
        self.declare_parameter('forward_chase_speed', 0.2)
        self.declare_parameter('search_angular_speed', 0.5)
        self.declare_parameter('min_distance_thresh', 30.0)  # Start moving forward
        self.declare_parameter('stop_distance_thresh', 27.0)  # Stop all movement
        self.declare_parameter('filter_value', 0.9)

        # Get parameters
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.min_distance_thresh = self.get_parameter('min_distance_thresh').get_parameter_value().double_value
        self.stop_distance_thresh = self.get_parameter('stop_distance_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value

        # Initialize state
        self.target_x = 0.0  # Lateral offset (cm)
        self.target_z = 0.0  # Distance (cm)
        self.lastrcvtime = time.time() - 10000

        # Control loop timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.get_logger().info(f'Tracking ArUco: x={self.target_x:.1f} cm, z={self.target_z:.1f} cm')
            if self.target_z <= self.stop_distance_thresh:
                # Stop all movement when too close
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.get_logger().info('ArUco marker reached, stopping.')
            else:
                # Normal tracking behavior
                if self.target_z > self.min_distance_thresh:
                    msg.linear.x = self.forward_chase_speed
                else:
                    msg.linear.x = 0.0
                msg.angular.z = -self.angular_chase_multiplier * self.target_x
        else:
            self.get_logger().info('ArUco marker lost')
            msg.angular.z = self.search_angular_speed
            msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        try:
            x, y, z = msg.data  # x, y, z in cm
            f = self.filter_value
            self.target_x = self.target_x * f + x * (1 - f)
            self.target_z = self.target_z * f + z * (1 - f)
            self.lastrcvtime = time.time()
        except Exception as e:
            self.get_logger().error(f'Error processing position: {e}')

def main(args=None):
    rclpy.init(args=args)
    follow_aruco = FollowAruco()
    rclpy.spin(follow_aruco)
    follow_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()