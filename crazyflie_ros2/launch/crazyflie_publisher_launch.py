import launch
import launch_ros
import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('crazyflie_ros2')


    crazyflie_node = launch_ros.actions.Node(
        package='crazyflie_ros2',
        executable='crazyflie_publisher')



    ld = LaunchDescription()
    ld.add_action(crazyflie_node)

    return ld