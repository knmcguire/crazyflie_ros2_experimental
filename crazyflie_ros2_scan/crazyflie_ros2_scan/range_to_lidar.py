

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.action import ActionServer

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from math import pi, copysign

from crazyflie_ros2_interfaces.action import MultiRangerScan
import tf_transformations

from rclpy.executors import MultiThreadedExecutor

import time

def is_close_to(current_value, goal_value, margin):
    if current_value > (goal_value - margin) and current_value < (goal_value + margin):
        return True
    else:
        return False

def wrap_to_pi(angle):
    if angle > pi:
        return (angle - 2.0 * pi)
    elif angle < -1.0 * pi:
        return (angle + 2.0 * pi)
    else:
        return angle



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
        self.get_logger().info(str(goal_handle.request.scan_angle - start_yaw ))
        self.get_logger().info(str(self.yaw ))

        while not is_close_to(wrap_to_pi(self.yaw - start_yaw), goal_handle.request.scan_angle, 0.05):
            self.get_logger().info(str(self.yaw ))
            # Send the command to start turning
            msg_cmd.angular.z = copysign(self.rot_vel_max, goal_handle.request.scan_angle)
            self.cmd_publisher.publish(msg_cmd)

            feedback_msg.current_angle = self.yaw
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        msg_cmd.angular.z = 0.0
        self.cmd_publisher.publish(msg_cmd)           

        result = MultiRangerScan.Result()
        result.laser_scan = [0.0, 0.0]

        goal_handle.succeed()
        return result

    def odom_subcribe_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        #self.get_logger().info('callback' + str(self.yaw ))

    def scan_subsribe_callback(self, msg):
        self.ranges = msg.ranges
        self.range_max = msg.range_max


def main(args=None):

    rclpy.init(args=args)

    try: 
        range_to_lidar = RangeToLidar()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(range_to_lidar)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            range_to_lidar.destroy_node()
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()
