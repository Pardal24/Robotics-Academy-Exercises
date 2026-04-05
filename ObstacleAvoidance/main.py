import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class CarNode(Node):

    def __init__(self):
        super().__init__('car_node')
        self.steer_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        # The field ranges in sensor_msgs/msg/LaserScan contains the measured distances expressed in meters.
        self.laser_sub_ = self.create_subscription(LaserScan, '/f1/laser/scan', self.listener_callback, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        #msg.data = 'Hello World: %d' % self.i
        #self.steer_publisher_.pub(msg)
        self.get_logger().info('Publishing')
    
    def listener_callback(self, msg):
        self.get_logger().info('Receiving')
    
    def parse_laser_data(laser_data):
        # Parses the LaserData object and returns a tuple with two lists:
        # 1. List of  polar coordinates, with (distance, angle) tuples,
        #    where the angle is zero at the front of the robot and increases to the left.
        # 2. List of cartesian (x, y) coordinates, following the ref. system noted below.

        # Note: The list of laser values MUST NOT BE EMPTY.

        laser_polar = []  # Laser data in polar coordinates (dist, angle)
        laser_xy = []  # Laser data in cartesian coordinates (x, y)
        for i in range(180):
            # i contains the index of the laser ray, which starts at the robot's right
            # The laser has a resolution of 1 ray / degree
            #
            #                (i=90)
            #                 ^
            #                 |x
            #             y   |
            # (i=180)    <----R      (i=0)

            # Extract the distance at index i
            dist = laser_data.values[i]
            # The final angle is centered (zeroed) at the front of the robot.
            angle = math.radians(i - 90)
            laser_polar += [(dist, angle)]
            # Compute x, y coordinates from distance and angle
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            laser_xy += [(x, y)]
        return laser_polar, laser_xy

    def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

        # robotx, roboty are the absolute coordinates of the robot
        # robott is its absolute orientation
        # Convert to relatives
        dx = x_abs - robotx
        dy = y_abs - roboty

        # Rotate with current angle
        x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
        y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

        return x_rel, y_rel



def main(args=None):
    #rclpy.init(args=args)

    car_node = CarNode()

    rclpy.spin(car_node)

    #car_node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()