from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    perception_config = os.path.join(
        get_package_share_directory("aljnump_pnp"), "config", "applepnp.yaml"
    )

    ld = []
    perception_node = Node(
        package="aljnump_pnp",
        executable="pnp_perception",
        name="pnp_perception_node",
        output="screen",
        parameters=[perception_config],
    )
    ld.append(perception_node)

    return LaunchDescription(ld)
