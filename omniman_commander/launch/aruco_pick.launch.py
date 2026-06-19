import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


# ArUco-driven pick-and-place.
# Prerequisites:
#   - nxp_omniman_launch.py running on the NUC (starts camera + aruco_tracker)
#   - Arm positioned so the camera can see the marker before launching this
#
#   ros2 launch omniman_commander aruco_pick.launch.py
def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .to_moveit_configs()
    )

    aruco_yaml = os.path.join(
        get_package_share_directory("omniman_commander"),
        "parameter",
        "aruco_pick.yaml",
    )

    commander_aruco = Node(
        package="omniman_commander",
        executable="commander_aruco",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            aruco_yaml,
        ],
    )

    return LaunchDescription([commander_aruco])