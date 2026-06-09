import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    use_trajectory = (
        LaunchConfiguration("use_trajectory").perform(context).lower() == "true"
    )

    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")

    if use_trajectory:
        servo_yaml["command_out_type"] = "trajectory_msgs/JointTrajectory"
        servo_yaml["command_out_topic"] = "/arm_controller/joint_trajectory"
    else:
        servo_yaml["command_out_type"] = "std_msgs/Float64MultiArray"
        servo_yaml["command_out_topic"] = "/arm_group_position_controller/commands"

    servo_params = {"moveit_servo": servo_yaml}

    rviz_config_file = (
        get_package_share_directory("moveit_servo") + "/config/omniman_servo_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
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
        name="joy_node",
        output="screen",
    )

    joy_to_gripper_node = Node(
        package="moveit_servo",
        executable="joy_to_gripper.py",
        name="joy_to_gripper",
        output="screen",
    )

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
    )

    return [
        # rviz_node,
        servo_node,
        joy_node,
        joy_to_gripper_node,
        container,
    ]


def generate_launch_description():
    use_trajectory_arg = DeclareLaunchArgument(
        "use_trajectory",
        default_value="false",
        description="false -> Float64MultiArray to JointGroupPositionController. "
                    "true -> JointTrajectory to JointTrajectoryController.",
    )

    return LaunchDescription(
        [
            use_trajectory_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
