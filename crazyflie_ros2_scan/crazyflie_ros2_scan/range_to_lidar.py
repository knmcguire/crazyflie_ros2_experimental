

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class RangeToLidar(Node):
    def __init__(self):
        super().__init__('range_to_lidar')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_subcribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_subsribe_callback, 10)

        self.yaw
        self.range
        self.range_max

    def odom_subcribe_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def scan_subsribe_callback(self, msg):
        self.ranges = msg.ranges
        self.range_max = msg.range_max

def main(args=None):

    rclpy.init(args=args)
    range_to_lidar = RangeToLidar()
    rclpy.spin(range_to_lidar)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
