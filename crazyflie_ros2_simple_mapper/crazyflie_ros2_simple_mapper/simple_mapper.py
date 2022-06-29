import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import tf_transformations

class SimpleMapper(Node):
    def __init__(self):
        super().__init__('simple_mapper')
        self.odom_subsriber = self.create_subscription(Odometry, '/odom', self.odom_subcribe_callback, 10)
        self.x_global = 0.0
        self.y_global = 0.0
        self.z_global = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def odom_subcribe_callback(self, msg):
        self.x_global = msg.pose.pose.position.x
        self.y_global = msg.pose.pose.position.y
        self.z_global = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        


def main(args=None):

    rclpy.init(args=args)
    simple_mapper = SimpleMapper()
    rclpy.spin(simple_mapper)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
