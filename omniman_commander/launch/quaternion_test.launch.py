from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="omniman_moveit_config").to_moveit_configs()
    )

    position_node = Node(
        package="omniman_commander",
        executable="quaternion_test",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([position_node])