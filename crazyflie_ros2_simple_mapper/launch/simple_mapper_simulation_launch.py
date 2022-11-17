import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    simple_mapper_node = launch_ros.actions.Node(
        package="crazyflie_ros2_simple_mapper", executable="simple_mapper"
    )

    pkg_share = get_package_share_directory("crazyflie_ros2_simulation")

    simulation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot_launch.py")
        )
    )

    # simulation_node
    ld = LaunchDescription()
    ld.add_action(simulation_node)
    ld.add_action(simple_mapper_node)

    return ld
