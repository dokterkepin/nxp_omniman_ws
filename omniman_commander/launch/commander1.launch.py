import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .to_moveit_configs()
    )

    poses_yaml = os.path.join(
        get_package_share_directory("omniman_commander"),
        "parameter",
        "poses.yaml",
    )

    commander1 = Node(
        package="omniman_commander",
        executable="commander1",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            poses_yaml,          # edit poses.yaml to change targets — no recompile needed
        ],
    )

    return LaunchDescription([commander1])
