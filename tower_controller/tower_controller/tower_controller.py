#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

class TowerController(Node):
    def __init__(self):
        super().__init__('tower_controller_node')

        # ROS2 Parameters
        self.declare_parameter('person_topic', '/user_coordinates')
        self.declare_parameter('cmd_topic', '/tower_yaw/cmd_pos')
        self.declare_parameter('min_angle_deg', -90.0)
        self.declare_parameter('max_angle_deg',  90.0)
        self.declare_parameter('smoothing_factor', 0.2)

        self.person_topic_ = self.get_parameter('person_topic').value
        self.cmd_topic_    = self.get_parameter('cmd_topic').value
        self.min_angle_deg_ = float(self.get_parameter('min_angle_deg').value)
        self.max_angle_deg_ = float(self.get_parameter('max_angle_deg').value)
        self.smoothing_factor_ = float(self.get_parameter('smoothing_factor').value)

        self.min_angle_rad_ = math.radians(self.min_angle_deg_)
        self.max_angle_rad_ = math.radians(self.max_angle_deg_)


        self.cmd_publisher_ = self.create_publisher(Float64, self.cmd_topic_, 10)

        self.person_subscriber_ = self.create_subscription(
            PointStamped,
            self.person_topic_,
            self.person_callback,
            10
        )
        self.current_cmd_rad_ = 0.0

    def person_callback(self, msg: PointStamped):

        x = msg.point.x  
        z = msg.point.z  

        # Compute the camera tower yaw command from person coordinates
        yaw_rad = -math.atan2(x, z)  # x<0 => right
        # Limit the yaw command to the configured physical range
        yaw_rad_clamped = max(self.min_angle_rad, min(self.max_angle_rad, yaw_rad))

        self.current_cmd_rad_ = (1.0 - self.smoothing_factor_) * self.current_cmd_rad_ + self.smoothing_factor_ * yaw_rad_clamped

        # Publish
        msg_out = Float64()
        msg_out.data = self.current_cmd_rad_
        self.cmd_publisher_.publish(msg_out)     

def main(args=None):
    rclpy.init(args=args)
    node = TowerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
