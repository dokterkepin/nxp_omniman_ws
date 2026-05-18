import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="omniman_moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("omniman_servo", "config/servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_footprint", "frame_id": "/world"}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.15,
        }],
    )

    # Launch a standalone Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'debug'],
    )

    return LaunchDescription([servo_node, container, joy_node])
