import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf_transformations

class SimpleMapper(Node):
    def __init__(self):
        super().__init__('simple_mapper')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_subcribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_subsribe_callback, 10)
        self.position =  [0.0, 0.0, 0.0]
        self.angles =  [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]

    def odom_subcribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]

    def scan_subsribe_callback(self, msg):
        self.ranges = msg.ranges


def main(args=None):

    rclpy.init(args=args)
    simple_mapper = SimpleMapper()
    rclpy.spin(simple_mapper)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
