from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Load the nxp_omniman MoveIt Configuration.
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="omniman_moveit_config").to_moveit_configs()
    )

    # Run the MTC node with robot_description and robot_description_semantic.
    hello_moveit = Node(
        package="omniman_commander",
        executable="fira_mission_yolo_cpp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([hello_moveit])