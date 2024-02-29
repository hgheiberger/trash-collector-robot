#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class DrivePublisher(Node):

    def __init__(self):
        super().__init__('drive_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.5
        drive_msg.angular.z = 0.0
        self.publisher_.publish(drive_msg)
        #self.get_logger().info('Publishing drive command')


def main(args=None):
    rclpy.init(args=args)

    drive_publisher = DrivePublisher()

    rclpy.spin(drive_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
