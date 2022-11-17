import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("crazyflie_ros2_description")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/crazyflie.rviz")
    default_model_path = os.path.join(pkg_share, "urdf/crazyflie_description.urdf")
    urdf_model = LaunchConfiguration("urdf_model")
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": True, "robot_description": Command(["xacro ", urdf_model])}
        ],
        arguments=[default_model_path],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
    )

    ld = LaunchDescription()
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
