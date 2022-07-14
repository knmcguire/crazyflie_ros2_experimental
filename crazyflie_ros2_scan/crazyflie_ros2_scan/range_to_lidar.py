

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.action import ActionServer

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from crazyflie_ros2_interfaces.action import MultiRangerScan
import tf_transformations

import time

class RangeToLidar(Node):
    def __init__(self):
        super().__init__('range_to_lidar')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_subcribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_subsribe_callback, 10)
        self.action_server = ActionServer(self, MultiRangerScan, 'multi_ranger_scan', self.execute_callback)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.yaw = 0.0
        self.range = 0.0
        self.range_max = 0.0
        self.rot_vel_max = 0.5

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        start_yaw = self.yaw
        feedback_msg = MultiRangerScan.Feedback()

        msg_cmd = Twist()

        # Check the yaw
        self.get_logger().info(str(goal_handle.request.scan_angle - start_yaw))


        while self.yaw < (goal_handle.request.scan_angle - start_yaw):
            self.get_logger().info(str(self.yaw ))
            # Send the command to start turning
            msg_cmd.angular.z = self.rot_vel_max
            self.cmd_publisher.publish(msg_cmd)

            feedback_msg.current_angle = self.yaw
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        msg_cmd.angular.z = 0
        self.cmd_publisher.publish(msg_cmd)           

        result = MultiRangerScan.Result()
        result.laser_scan = [0.0, 0.0]

        goal_handle.succeed()
        return result

    def odom_subcribe_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.get_logger().info('callback' + str(self.yaw ))

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
