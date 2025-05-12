from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    tfparam_path = os.path.join(
        get_package_share_directory("aljnump_description"),
        "config",
        "intelrs",
        "camera_tf.yaml",
    )
    with open(tfparam_path, "r") as stream:
        tfparam = yaml.safe_load(stream)

    realsense_config_file = os.path.join(
        get_package_share_directory("aljnump_description"),
        "config",
        "intelrs",
        "realsense.yaml",
    )

    ld = []
    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_tool0_to_camera",
        output="screen",
        arguments=[
            str(tfparam["translation"]["x"]),
            str(tfparam["translation"]["y"]),
            str(tfparam["translation"]["z"]),
            str(tfparam["rotation"]["x"]),
            str(tfparam["rotation"]["y"]),
            str(tfparam["rotation"]["z"]),
            str(tfparam["rotation"]["w"]),
            tfparam["parent_frame_id"],
            tfparam["child_frame_id"],
        ],
    )
    ld.append(tf_node)

    # TODO: add camera node here

    return LaunchDescription(ld)
