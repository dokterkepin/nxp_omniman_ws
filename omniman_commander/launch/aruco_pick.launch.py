import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


# ArUco-driven pick-and-place.
# Prerequisite: robot bringup (nxp_omniman_launch.py) must already be running —
# it starts the camera driver with the correct frame_id and calibration.
#
#   ros2 launch omniman_commander aruco_pick.launch.py marker_size:=0.05
def generate_launch_description():
    marker_size = LaunchConfiguration("marker_size")
    marker_dict = LaunchConfiguration("marker_dict")

    args = [
        DeclareLaunchArgument("marker_size", default_value="0.05",
                              description="Physical marker side length in meters"),
        DeclareLaunchArgument("marker_dict", default_value="4X4_50",
                              description="ArUco dictionary (must match the printed marker)"),
    ]

    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .to_moveit_configs()
    )

    aruco_yaml = os.path.join(
        get_package_share_directory("omniman_commander"),
        "parameter",
        "aruco_pick.yaml",
    )

    aruco_tracker = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[{
            "cam_base_topic": "image_raw",
            "marker_size": marker_size,
            "marker_dict": marker_dict,
            "image_is_rectified": False,
            "output_frame": "",
        }],
        remappings=[
            ("/image_raw/camera_info", "/camera_info"),
        ],
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

    return LaunchDescription(args + [aruco_tracker, commander_aruco])