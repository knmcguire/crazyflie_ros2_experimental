import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E703')

def radians(degrees):  
    return degrees * math.pi / 180.0;

class CrazyfliePublisher(Node):

    def __init__(self, link_uri):
        super().__init__('crazyflie_publisher')
        self.publisher_ = self.create_publisher(Pose, 'pose', 10)
        self.tfbr_base = TransformBroadcaster(self)
        self.tfbr_cf = TransformBroadcaster(self)

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        print('connected')
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')


    def _disconnected():
        print('disconnected')

    def _connection_failed(self, link_uri, msg):
        print('connection_failed')

    def _connection_lost(self, link_uri, msg):
        print('connection_lost')

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()
        msg = Pose()

        x = data.get('stateEstimate.x')
        y = data.get('stateEstimate.y')
        z = data.get('stateEstimate.z')
        roll = radians(data.get('stabilizer.roll'))
        pitch = radians(-1.0 * data.get('stabilizer.pitch'))
        yaw = radians(data.get('stabilizer.yaw'))
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        self.publisher_.publish(msg)


        q_base = tf_transformations.quaternion_from_euler(0, 0, yaw)
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'map'
        t_base.child_frame_id = 'crazyflie_base'
        t_base.transform.translation.x = x
        t_base.transform.translation.y = y
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = q_base[0]
        t_base.transform.rotation.y = q_base[1]
        t_base.transform.rotation.z = q_base[2]
        t_base.transform.rotation.w = q_base[3]
        self.tfbr_base.sendTransform(t_base)

        t_cf = TransformStamped()
        q_cf = tf_transformations.quaternion_from_euler(roll, pitch, 0)
        t_cf.header.stamp = self.get_clock().now().to_msg()
        t_cf.header.frame_id = 'crazyflie_base'
        t_cf.child_frame_id = 'crazyflie'
        t_cf.transform.translation.x = 0.0
        t_cf.transform.translation.y = 0.0
        t_cf.transform.translation.z = z
        t_cf.transform.rotation.x = q_cf[0]
        t_cf.transform.rotation.y = q_cf[1]
        t_cf.transform.rotation.z = q_cf[2]
        t_cf.transform.rotation.w = q_cf[3]
        self.tfbr_cf.sendTransform(t_cf)

        

def main(args=None):

    cflib.crtp.init_drivers()

    rclpy.init(args=args)

    crazyflie_publisher = CrazyfliePublisher(uri)

    rclpy.spin(crazyflie_publisher)

    crazyflie_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()




