import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("master_sim"), "config", "csad.yaml"
    )

    sim = Node(
        package="master_sim",
        executable="mcsim_node",
        parameters=[config],
        output="screen",
    )

    return LaunchDescription([sim])