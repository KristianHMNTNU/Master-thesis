import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("master_sim")
    config = os.path.join(share, "config", "csad.yaml")
    rviz_config = os.path.join(share, "rviz", "sim.rviz")
    urdf_file = os.path.join(
        get_package_share_directory("cybership_description"),
        "urdf", "model", "drillship", "base.urdf.xacro",
    )
    robot_description = xacro.process_file(urdf_file).toxml()

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

    ned_to_enu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--roll", "3.14159265",
            "--yaw", "1.57079633",
            "--frame-id", "map",
            "--child-frame-id", "world",
        ],
    )

    model = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription([sim, ned_to_enu, rviz, model])