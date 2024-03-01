import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String


class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")
        
        ## will receive an x,y coord
        self.create_subscription(Point,'/trash_pixel_loc', self.trash_pixel_loc_callback, 10)
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(String, 'parking_error',10)
        self.data_logger = self.create_publisher(String, 'data',10)


        # May not exist on
        self.declare_parameter('parking_distance', 0.15)
        self.declare_parameter('velocity', 0.75)

        self.parking_distanace = self.get_parameter('parking_distance').get_parameter_value().double_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value

        # self.parking_distance = 0.15 if self.LineFollower else 0.5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.orientation = False

    
    def trash_pixel_loc_callback(self, msg):
        """
        Park car facing object given x_pos and y_pos
        """
        self.get_logger().info(f"Heard point: ({msg.x}, {msg.y})")
      
        self.relative_x = msg.x
        self.relative_y = msg.y
        L = 0.325 # Wheel base TODO Need to Change
        eta = math.atan2(self.relative_y, self.relative_x)
        cone_dist = math.sqrt(self.relative_x**2 + self.relative_y**2)
        self.get_logger().info("Cone Dist: " + str(cone_dist) + " x: " + str(self.relative_x) + " y: " + str(self.relative_y))
        setpoint = self.parking_distance
        threshold = .3 # meters
        
        # look_ahead = 0.5
        # steer_angle = math.atan(2*L*math.sin(eta)/look_ahead)
        # steer_angle = math.atan( (L*math.sin(eta)) / ((look_ahead/2) + (look_ahead* math.cos(eta))))

        look_ahead = 1 * self.VELOCITY
        steer_angle = math.atan(2*L*math.sin(eta)/look_ahead)
        #steer_angle = math.atan((L*math.sin(eta)) / ((look_ahead/2) + (look_ahead*math.cos(eta))))
        drive_msg = Twist()

        if self.orientation == False:
            if cone_dist > setpoint + threshold: # Cone too far
                # self.drive(self.VELOCITY, steer_angle)
                drive_msg.linear.x = self.VELOCITY
                drive_msg.angular.z = steer_angle
                self.drive_pub.publish(drive_msg)

            elif cone_dist < setpoint - threshold: # cone too close
                self.orientation = True
                # self.drive(-self.VELOCITY,-steer_angle)
                drive_msg.linear.x = -self.VELOCITY
                drive_msg.angular.z = -steer_angle
                self.drive_pub.publish(drive_msg)


            else: # cone within range
                # rospy.loginfo("Cone within range ")
                if abs(eta) > .15:
                    self.orientation = True
                    # self.drive(-self.VELOCITY, -steer_angle)
                    drive_msg.linear.x = -self.VELOCITY
                    drive_msg.angular.z = -steer_angle
                    self.drive_pub.publish(drive_msg)
                else:
                    self.drive(0,0)
                    drive_msg.linear.x = 0
                    drive_msg.angular.z = 0
                    self.drive_pub.publish(drive_msg)

        else:
            if (cone_dist > (setpoint - threshold) or (eta < 0.05)):
                self.orientation = False

            else:
                # self.drive(-self.VELOCITY, -steer_angle)
                drive_msg.linear.x = -self.VELOCITY
                drive_msg.angular.z = -steer_angle
                self.drive_pub.publish(drive_msg)

        self.error_publisher()



def main():
    rclpy.init()

    parking_controller = ParkingController()

    rclpy.spin(parking_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parking_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()