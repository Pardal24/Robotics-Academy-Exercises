import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class CarNode(Node):

    def __init__(self):
        super().__init__('car_node')
        self.steer_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
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


def main(args=None):
    #rclpy.init(args=args)

    car_node = CarNode()

    rclpy.spin(car_node)

    #car_node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()