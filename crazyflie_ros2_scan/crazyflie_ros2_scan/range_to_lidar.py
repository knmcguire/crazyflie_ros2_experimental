

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.action import ActionServer

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from crazyflie_ros2_interfaces.action import MultiRangerScan


class RangeToLidar(Node):
    def __init__(self):
        super().__init__('range_to_lidar')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_subcribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_subsribe_callback, 10)
        self._action_server = ActionServer(
            self,
            MultiRangerScan,
            'multi_ranger_scan',
            self.execute_callback)

        self.yaw = 0.0
        self.range = 0.0
        self.range_max = 0.0


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = MultiRangerScan.Result()
        goal_handle.succeed()
        return result

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
