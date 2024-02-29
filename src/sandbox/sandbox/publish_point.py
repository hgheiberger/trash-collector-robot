#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class PointPublisher(Node):

    def __init__(self):
        super().__init__('drive_publisher')
        self.publisher_ = self.create_publisher(Point, '/trash_pixel_loc', 10)
        self.marker_pub = rclpy.create_publisher(Marker, "/trash_pixel_marker", 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        point_msg = Point()
        point_msg.x = 3.0
        point_msg.y = 2.0
        point_msg.z = 0.0
        self.publisher_.publish(point_msg)
        self.get_logger().info('Publishing point command')
        self.draw_marker(point_msg.x, point_msg.y, "base_link")

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    point_publisher = PointPublisher()

    rclpy.spin(point_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
