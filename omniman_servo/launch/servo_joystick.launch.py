import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "nxp_omniman", package_name="omniman_moveit_config"
    ).to_moveit_configs()

    servo_yaml = load_yaml("omniman_servo", "config/servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Merge all parameters into a single dict
    all_params = {}
    all_params.update(servo_params)
    all_params.update(moveit_config.robot_description)
    all_params.update(moveit_config.robot_description_semantic)
    all_params.update(moveit_config.robot_description_kinematics)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[all_params],
        output="screen",
    )

    # Joystick input + converter to Servo twist commands
    container = ComposableNodeContainer(
        name="servo_joy_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="joy_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    return LaunchDescription([servo_node, container])
