import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    onrobotsg_config_file = os.path.join(
        get_package_share_directory("aljnump_description"),
        "config",
        "sg",
        "hardware_parameters.yaml",
    )

    ld = []
    ld.append(
        Node(
            package="onrobotsg",
            executable="onrobotsg_service",
            name="onrobotsg_node",
            parameters=[onrobotsg_config_file],
            output="screen",
        ),
    )

    return LaunchDescription(ld)
