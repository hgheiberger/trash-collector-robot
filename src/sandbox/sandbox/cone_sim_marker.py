#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from tf_transformations import euler_from_quaternion

import tf2_ros

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped , Point

class SimMarker(Node):
    """
    Rosnode for handling simulated cone. Listens for clicked point
    in rviz and publishes a marker. Publishes position of cone
    relative to robot for Parking Controller to park in front of.
    """
    def __init__(self):
        super().__init__('cone_sim_marker')


        # Subscribe to clicked point messages from rviz    
        self.subscription = self.create_subscription(
            PoseStamped ,
            '/goal_pose',
            self.clicked_callback,
            10
        )


        self.message_x = None
        self.message_y = None
        self.message_frame = "map"

        self.cone_pub = self.create_publisher(Point, '/trash_pixel_loc', 1)
        self.marker_pub = self.create_publisher(Marker, '/cone_marker', 1)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.timer_period = 0.1 # seconds (TODO: to increase how fast publishing is without breaking threading)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
   
    def publish_cone(self):
        """
        Publish the relative location of the cone
        """
        # Find out most recent relative location of cone
        if self.message_x is None:
            return
        
        self.get_logger().info("Checkpoint 1")

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.message_frame, rclpy.time.Time())
        except:
            return
        
        #self.get_logger().info("Checkpoint 2")
        # Using relative transformations, convert cone in whatever frame rviz
        # was in to cone in base link (which is the control frame)

        rot = transform.transform.rotation
        quat = [rot.x, rot.y, rot.z, rot.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        msg_frame_pos = transform.transform.translation


        #self.get_logger().info("Checkpoint 3")
        cone_relative_baselink_x =\
            msg_frame_pos.x+np.cos(yaw)*self.message_x-np.sin(yaw)*self.message_y 
        cone_relative_baselink_y =\
            msg_frame_pos.y+np.cos(yaw)*self.message_y+np.sin(yaw)*self.message_x
        
        #self.get_logger().info("Checkpoint 4")

        # Publish relative cone location
        relative_cone = Point()
        relative_cone.x = cone_relative_baselink_x
        relative_cone.y = cone_relative_baselink_y
        self.get_logger().info("%f" % relative_cone.x_pos)
        self.get_logger().info("%f" % relative_cone.y_pos)

        self.cone_pub.publish(relative_cone)

        

    def draw_marker(self):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.message_x
        marker.pose.position.y = self.message_y
        self.marker_pub.publish(marker)

        
    def clicked_callback(self, msg):
        print("callback called")
        # Store clicked point in the map frame
        transform = self.tf_buffer.lookup_transform(
                self.message_frame, msg.header.frame_id, rclpy.time.Time())
        

       

        rot = transform.transform.rotation
        quat = [rot.x, rot.y, rot.z, rot.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        
        msg_frame_pos = transform.transform.translation
        
        self.message_x = \
            msg_frame_pos.x+np.cos(yaw)*msg.pose.position.point.x\
            -np.sin(yaw)*msg.pose.position.y
        
        self.message_y = \
            msg_frame_pos.y+np.cos(yaw)*msg.pose.position.point.y\
            +np.sin(yaw)*msg.pose.position.x
        
        # Draw a marker for visualization
        self.draw_marker()
        print("callback ended")

    def timer_callback(self):
        self.publish_cone()


def main(args=None):
    rclpy.init(args=args)

    sim_marker = SimMarker()
    rclpy.spin(sim_marker)
    
    sim_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()