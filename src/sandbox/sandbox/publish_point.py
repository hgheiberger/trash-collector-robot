#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class PointPublisher(Node):

    def __init__(self):
        super().__init__('drive_publisher')
        self.publisher_ = self.create_publisher(Point, '/trash_pixel_loc', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        point_msg = Point()
        point_msg.x = 3
        point_msg.y = 2
        point_msg.z = 0.0
        self.publisher_.publish(point_msg)
        self.get_logger().info('Publishing point command')


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
