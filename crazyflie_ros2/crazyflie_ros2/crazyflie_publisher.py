import math
from math import pi
from typing import Any

import cflib.crtp  # noqa
import rclpy
import tf_transformations
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from tf2_ros import TransformBroadcaster

URI = uri_helper.uri_from_env(default="radio://0/40/2M/E7E7E7E703")
FLYING = True


def radians(degrees: float) -> float:
    return degrees * math.pi / 180.0


class CrazyfliePublisher(Node):
    def __init__(self, link_uri: str):
        super().__init__("crazyflie_publisher")
        # self.pose_publisher = self.create_publisher(Pose, 'pose', 10)
        self.range_publisher = self.create_publisher(Range, "zrange", 10)
        self.laser_publisher = self.create_publisher(LaserScan, "scan", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self._cf = Crazyflie(rw_cache="./cache")
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)

        self.ranges = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.create_timer(1.0 / 30.0, self.publish_laserscan_data)
        if FLYING:
            timer_period = 0.1  # seconds
            self.create_timer(timer_period, self.send_hover_command)
            self.hover = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "height": 0.3}
            self._cf.commander.send_hover_setpoint(
                self.hover["x"],
                self.hover["y"],
                self.hover["yaw"],
                self.hover["height"],
            )
        self._logger = self.get_logger()
        self.target_twist = None

    def publish_laserscan_data(self) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.range_min = 0.01
        msg.range_max = 3.49
        msg.ranges = self.ranges
        msg.angle_min = 0.5 * 2 * pi
        msg.angle_max = -0.5 * 2 * pi
        msg.angle_increment = -1.0 * pi / 2
        self.laser_publisher.publish(msg)

    def send_hover_command(self) -> None:
        hover_height = self.hover["height"] + self.hover["z"] * 0.1
        self._cf.commander.send_hover_setpoint(
            self.hover["x"], self.hover["y"], self.hover["yaw"], hover_height
        )
        self.hover["height"] = hover_height

    def _connected(self) -> None:
        self._logger.info("Connected!")
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=100)
        self._lg_stab.add_variable("stateEstimate.x", "float")
        self._lg_stab.add_variable("stateEstimate.y", "float")
        self._lg_stab.add_variable("stateEstimate.z", "float")
        self._lg_stab.add_variable("stabilizer.roll", "float")
        self._lg_stab.add_variable("stabilizer.pitch", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")

        self._lg_range = LogConfig(name="Range", period_in_ms=100)
        self._lg_range.add_variable("range.zrange", "uint16_t")
        self._lg_range.add_variable("range.front", "uint16_t")
        self._lg_range.add_variable("range.right", "uint16_t")
        self._lg_range.add_variable("range.left", "uint16_t")
        self._lg_range.add_variable("range.back", "uint16_t")

        try:
            self._cf.log.add_config(self._lg_stab)
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            self._lg_stab.start()
            self._cf.log.add_config(self._lg_range)
            self._lg_range.data_received_cb.add_callback(self._range_log_data)
            self._lg_range.error_cb.add_callback(self._range_log_error)
            self._lg_range.start()
        except KeyError as e:
            self._logger.error(
                "Could not start log configuration,"
                "{} not found in TOC".format(str(e))
            )
        except AttributeError:
            self._logger.error(
                "Could not add Stabilizer log config, bad configuration."
            )

    def _disconnected(self) -> None:
        self._logger.info("disconnected")

    def _connection_failed(self) -> None:
        self._logger.info("connection_failed")

    def _connection_lost(self) -> None:
        self._logger.info("connection_lost")

    def _range_log_error(self, logconf: rclpy.Parameter, msg: Range) -> None:
        """Callback from the log API when an error occurs"""
        self._logger.error("Error when logging %s: %s" % (logconf.name, msg))

    def _range_log_data(self, timestamp: str, data: dict, logconf: LogConfig) -> None:
        """Callback from a log API when data arrives"""
        for name, value in data.items():
            self._logger.info(f"{name}: {value:3.3f} ", end="")
        self._logger.info()

        t_range = TransformStamped()
        q = tf_transformations.quaternion_from_euler(0, radians(90), 0)
        t_range.header.stamp = self.get_clock().now().to_msg()
        t_range.header.frame_id = "base_link"
        t_range.child_frame_id = "crazyflie_flowdeck"
        t_range.transform.rotation.x = q[0]
        t_range.transform.rotation.y = q[1]
        t_range.transform.rotation.z = q[2]
        t_range.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t_range)

        zrange = float(data.get("range.zrange")) / 1000.0
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "crazyflie_flowdeck"
        msg.field_of_view = radians(4.7)
        msg.radiation_type = Range().INFRARED
        msg.min_range = 0.01
        msg.max_range = 3.5
        msg.range = zrange
        # self.range_publisher.publish(msg)

        max_range = 3.49
        front_range = float(data.get("range.front")) / 1000.0
        left_range = float(data.get("range.left")) / 1000.0
        back_range = float(data.get("range.back")) / 1000.0
        right_range = float(data.get("range.right")) / 1000.0
        if front_range > max_range:
            front_range = float("inf")
        if left_range > max_range:
            left_range = float("inf")
        if right_range > max_range:
            right_range = float("inf")
        if back_range > max_range:
            back_range = float("inf")
        self.ranges = [back_range, left_range, front_range, right_range, back_range]

    def cmd_vel_callback(self, twist: Twist) -> None:
        self.target_twist = twist

        self.hover["x"] = twist.linear.x
        self.hover["y"] = twist.linear.y
        self.hover["z"] = twist.linear.z
        self.hover["yaw"] = -1 * math.degrees(twist.angular.z)

    def _stab_log_error(self, logconf: LogConfig, msg: Any) -> None:
        """Callback from the log API when an error occurs"""
        self._logger.error("Error when logging %s: %s" % (logconf.name, msg))

    def _stab_log_data(self, timestamp: str, data: Any, logconf: LogConfig) -> None:
        """Callback from a log API when data arrives"""
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()

        msg = Odometry()

        x = data.get("stateEstimate.x")
        y = data.get("stateEstimate.y")
        z = data.get("stateEstimate.z")
        roll = radians(data.get("stabilizer.roll"))
        pitch = radians(-1.0 * data.get("stabilizer.pitch"))
        yaw = radians(data.get("stabilizer.yaw"))
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.odom_publisher.publish(msg)

        q_base = tf_transformations.quaternion_from_euler(0, 0, yaw)
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = "odom"
        t_base.child_frame_id = "base_footprint"
        t_base.transform.translation.x = x
        t_base.transform.translation.y = y
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = q_base[0]
        t_base.transform.rotation.y = q_base[1]
        t_base.transform.rotation.z = q_base[2]
        t_base.transform.rotation.w = q_base[3]
        self.tf_broadcaster.sendTransform(t_base)

        t_odom = TransformStamped()
        t_odom.header.stamp = self.get_clock().now().to_msg()
        t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_footprint"
        q_odom = tf_transformations.quaternion_from_euler(0, 0, 0)
        t_odom.transform.translation.x = 0.0
        t_odom.transform.translation.y = 0.0
        t_odom.transform.translation.z = 0.1
        t_odom.transform.rotation.x = q_odom[0]
        t_odom.transform.rotation.y = q_odom[1]
        t_odom.transform.rotation.z = q_odom[2]
        t_odom.transform.rotation.w = q_odom[3]

        # self.tf_broadcaster.sendTransform(t_odom)

        t_cf = TransformStamped()
        q_cf = tf_transformations.quaternion_from_euler(roll, pitch, 0)
        t_cf.header.stamp = self.get_clock().now().to_msg()
        t_cf.header.frame_id = "base_footprint"
        t_cf.child_frame_id = "base_link"
        t_cf.transform.translation.x = 0.0
        t_cf.transform.translation.y = 0.0
        t_cf.transform.translation.z = z
        t_cf.transform.rotation.x = q_cf[0]
        t_cf.transform.rotation.y = q_cf[1]
        t_cf.transform.rotation.z = q_cf[2]
        t_cf.transform.rotation.w = q_cf[3]
        self.tf_broadcaster.sendTransform(t_cf)


def main(args=None) -> None:
    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_publisher = CrazyfliePublisher(URI)
    rclpy.spin(crazyflie_publisher)
    crazyflie_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
