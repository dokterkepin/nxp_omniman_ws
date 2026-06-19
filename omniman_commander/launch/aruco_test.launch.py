from launch import LaunchDescription
from launch_ros.actions import Node


# ArUco detection test — run on the remote PC while robot bringup runs on the NUC.
# Prerequisite: nxp_omniman_launch.py must be running on the NUC (publishes /image_raw).
#
#   ros2 launch omniman_commander aruco_test.launch.py
#   ros2 topic echo /aruco_detections
def generate_launch_description():
    aruco_tracker = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[{
            "cam_base_topic": "image_raw",
            "marker_size": 0.04,
            "marker_dict": "4X4_50",
            "image_is_rectified": False,
        }],
        remappings=[
            ("/image_raw/camera_info", "/camera_info"),
        ],
    )

    return LaunchDescription([aruco_tracker])