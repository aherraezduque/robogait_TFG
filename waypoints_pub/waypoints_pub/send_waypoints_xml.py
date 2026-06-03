#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET

from custom_msgs.msg import ActorTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_from_xml')

        # ROS parameters
        self.declare_parameter('xml_file', '')
        self.declare_parameter('topic_name', '/actor_cmd_script')
        self.declare_parameter('publish_period', 0.2)

        self.xml_file_ = self.get_parameter('xml_file').value
        self.topic_name_ = self.get_parameter('topic_name').value
        self.publish_period_ = self.get_parameter('publish_period').value


        if not self.xml_file_:
            raise ValueError('Parameter "xml_file" must be set')
        
        # Load waypoints from XML file 
        self.waypoints_ = self.load_waypoints(self.xml_file_)
        self.get_logger().info(f'Loaded {len(self.waypoints_)} waypoints from {self.xml_file_}')

        # Create Publisher
        self.publisher_ = self.create_publisher(ActorTrajectoryPoint, self.topic_name_, 10)

        # Create Timer to publish waypoints
        self.current_index_ = 0
        self.timer_ = self.create_timer(self.publish_period_, self.publish_next)

    def load_waypoints(self, xml_file):
        waypoints = []

        tree = ET.parse(xml_file)
        root = tree.getroot()

        for waypoint_elem in root.findall('waypoint'):
            waypoints.append(self.create_waypoint_msg(waypoint_elem))

        return waypoints
    
    def create_waypoint_msg(self, waypoint_element):
            
        msg = ActorTrajectoryPoint()

        position_element = waypoint_element.find('pose/position')
        orientation_element = waypoint_element.find('pose/orientation')
        time_element = waypoint_element.find('t')

        if position_element is None:
            raise ValueError('Waypoint is missing pose/position element')
        
        if orientation_element is None:
            raise ValueError('Waypoint is missing pose/orientation element')

        if time_element is None: 
            raise ValueError('Waypoint is missing t element')

        pose = Pose()
        
        pose.position = Point(
            x = float(position_element.get('x')),
            y = float(position_element.get('y')),
            z = float(position_element.get('z'))
        )
        
        pose.orientation = Quaternion(
            x = float(orientation_element.get('x')),
            y = float(orientation_element.get('y')),
            z = float(orientation_element.get('z')),
            w = float(orientation_element.get('w'))
        )
        
        msg.pose = pose
        msg.t = float(time_element.text)
        msg.clear = waypoint_element.get('clear', 'false').lower() == 'true'

        return msg

    def publish_next(self):
        if self.current_index_ >= len(self.waypoints_):
            self.get_logger().info('All waypoints have been published')
            self.timer_.cancel()
            return

        msg = self.waypoints_[self.current_index_]
        self.publisher_.publish(msg)

        """ self.get_logger().info(f'Published waypoint {self.current_index_}: '
                               f'pos=({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}), '
                               f't={msg.t}, clear={msg.clear}') """
        self.current_index_ += 1


def main(args=None):
    rclpy.init(args=args)

    node = WaypointPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
