from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare("leader_ros2_control")

    robot_controllers = PathJoinSubstitution(
        [pkg_path, "config", "controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [pkg_path, "config", "ros2_control_config.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_path, "description", "leader_ros2_control.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--param-file", robot_controllers],
    )

    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_position_controller,
            delay_rviz,
        ]
    )
