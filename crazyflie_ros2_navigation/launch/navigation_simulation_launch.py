"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy import logging

logger = logging.get_logger(__name__)


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Specify the actions
    logger.info(
        os.path.join(
            get_package_share_directory("crazyflie_ros2_simple_mapper"), "map.yaml"
        )
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "slam": "False",
            "map": os.path.join(
                get_package_share_directory("crazyflie_ros2_simple_mapper"), "map.yaml"
            ),
            "use_sim_time": "true",
            "params_file": os.path.join(
                get_package_share_directory("crazyflie_ros2_navigation"),
                "params",
                "nav2_params.yaml",
            ),
            "autostart": "true",
            "use_composition": "true",
        }.items(),
    )
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        launch_arguments={
            "rviz_config": os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz")
        }.items(),
    )

    pkg_share = get_package_share_directory("crazyflie_ros2_slam")

    simulation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share, "launch", "slam_toolbox_mapping_simulation_launch.py"
            )
        )
    )

    ld = LaunchDescription()

    ld.add_action(bringup_cmd)
    ld.add_action(simulation_node)
    ld.add_action(rviz_cmd)

    return ld
