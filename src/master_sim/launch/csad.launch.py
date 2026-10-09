import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("master_sim")
    config = os.path.join(share, "config", "csad.yaml")
    rviz_config = os.path.join(share, "rviz", "sim.rviz")

    sim = Node(
        package="master_sim",
        executable="mcsim_node",
        parameters=[config],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([sim, rviz])