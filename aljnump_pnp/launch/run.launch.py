from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("aljnump_pnp"), "config", "run.yaml"
    )

    ld = []
    runnode = Node(
        package="aljnump_pnp",
        executable="pnp_real_run",
        name="pnp_real_run_node",
        output="screen",
        parameters=[config],
    )
    ld.append(runnode)

    return LaunchDescription(ld)
