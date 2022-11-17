import launch_ros
from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    crazyflie_node = launch_ros.actions.Node(
        package="crazyflie_ros2", executable="crazyflie_publisher"
    )
    ld = LaunchDescription()
    ld.add_action(crazyflie_node)

    return ld
