import time
from math import copysign, degrees, pi
from typing import Any

import rclpy
import tf_transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from crazyflie_ros2_interfaces.action import MultiRangerScan


def is_close_to(current_value: float, goal_value: float, margin: float) -> bool:
    if (goal_value - margin) < current_value < (goal_value + margin):
        return True
    else:
        return False


def wrap_to_pi(angle: float) -> float:
    if angle > pi:
        return angle - 2.0 * pi
    elif angle < -1.0 * pi:
        return angle + 2.0 * pi
    else:
        return angle


class RangeToLidar(Node):
    def __init__(self):
        super().__init__("range_to_lidar")
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_subscribe_callback, 10
        )
        self.ranges_subscriber = self.create_subscription(
            LaserScan, "/multiranger_scan", self.scan_subscribe_callback, 10
        )
        self.action_server = ActionServer(
            self, MultiRangerScan, "multi_ranger_scan", self.execute_callback
        )
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.lidar_scan_publisher = self.create_publisher(LaserScan, "/scan", 10)

        self.yaw = 0.0
        self.range = 0.0
        self.range_max = 0.0
        self.rot_vel_max = 0.5 * pi

        self.lidar_scan = [float("inf")] * 361

        self.stamp = None
        self.ranges = None

        self.__logger = self.get_logger()

    def execute_callback(self, goal_handle: Any) -> MultiRangerScan.Result:
        self.__logger.info("Executing goal...")

        start_yaw = self.yaw
        feedback_msg = MultiRangerScan.Feedback()

        msg_cmd = Twist()

        # Check the yaw
        self.get_logger().info(str(goal_handle.request.scan_angle - start_yaw))
        self.get_logger().info(str(self.yaw))

        while not is_close_to(
            wrap_to_pi(self.yaw - start_yaw), goal_handle.request.scan_angle, 0.05
        ):
            self.get_logger().info(str(self.yaw))
            # Send the command to start turning
            msg_cmd.angular.z = copysign(
                self.rot_vel_max, goal_handle.request.scan_angle
            )
            self.cmd_publisher.publish(msg_cmd)

            feedback_msg.current_angle = self.yaw
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        while not is_close_to(self.yaw, start_yaw, 0.05):
            msg_cmd.angular.z = -1.0 * copysign(
                self.rot_vel_max, goal_handle.request.scan_angle
            )
            self.cmd_publisher.publish(msg_cmd)
            feedback_msg.current_angle = self.yaw
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        msg = LaserScan()
        msg.header.stamp = self.stamp
        msg.header.frame_id = "base_link"
        msg.range_min = 0.01
        msg.range_max = 3.5
        msg.ranges = self.lidar_scan
        msg.angle_min = 0.5 * 2 * pi
        msg.angle_max = -0.5 * 2 * pi
        msg.angle_increment = -2.0 * pi / 360.0
        self.lidar_scan_publisher.publish(msg)

        msg_cmd.angular.z = 0.0
        self.cmd_publisher.publish(msg_cmd)

        result = MultiRangerScan.Result()
        result.laser_scan = self.lidar_scan
        self.lidar_scan = [float("inf")] * 361

        goal_handle.succeed()
        return result

    def odom_subscribe_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        # self.get_logger().info('callback' + str(self.yaw ))

    def scan_subscribe_callback(self, msg: LaserScan) -> None:
        self.ranges = msg.ranges
        self.range_max = msg.range_max
        self.stamp = msg.header.stamp
        self.fill_lidar_scan()

    def fill_lidar_scan(self) -> None:
        self.lidar_scan[int(degrees(wrap_to_pi(self.yaw + pi)))] = self.ranges[2]
        self.lidar_scan[int(degrees(self.yaw))] = self.ranges[0]
        self.lidar_scan[int(degrees(wrap_to_pi(self.yaw + 0.5 * pi)))] = self.ranges[1]
        self.lidar_scan[int(degrees(wrap_to_pi(self.yaw + 1.5 * pi)))] = self.ranges[3]


def main(args=None) -> None:
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


if __name__ == "__main__":
    main()
