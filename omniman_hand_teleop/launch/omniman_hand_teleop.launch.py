import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def launch_setup(context, *args, **kwargs):
    use_trajectory = LaunchConfiguration("use_trajectory").perform(context).lower() == "true"
    show_window   = LaunchConfiguration("show_window").perform(context).lower() == "true"

    pkg_share  = get_package_share_directory("omniman_hand_teleop")
    teleop_params = os.path.join(pkg_share, "config", "teleop.yaml")
    config_dir = os.path.join(pkg_share, "config", "pose_tracking")
    rviz_config = os.path.join(pkg_share, "config", "rviz", "omniman_hand_teleop.rviz")

    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("omniman_hand_teleop")
        .yaml(os.path.join(config_dir, "pose_tracking_settings.yaml"))
        .yaml(os.path.join(config_dir, "omniman_pose_tracking.yaml"))
        .to_dict()
    }

    # Match servo output to whichever arm controller is active on the robot side.
    # Use the same use_trajectory arg as nxp_omniman_launch.py and servo_example.launch.py.
    s = servo_params["moveit_servo"]
    if use_trajectory:
        s["command_out_type"]  = "trajectory_msgs/JointTrajectory"
        s["command_out_topic"] = "/arm_controller/joint_trajectory"
        s["publish_joint_velocities"] = True
    else:
        s["command_out_type"]  = "std_msgs/Float64MultiArray"
        s["command_out_topic"] = "/arm_group_position_controller/commands"
        s["publish_joint_velocities"] = False

    pose_tracking_node = Node(
        package="omniman_hand_teleop",
        executable="omniman_pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    mediapipe_node = Node(
        package="omniman_hand_teleop",
        executable="hand_pose_publisher_node.py",
        name="hand_pose_publisher_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mediapipe")),
        parameters=[
            teleop_params,
            {
                "show_window":    show_window,
                "planning_frame": "base_link",
            },
        ],
    )

    return [pose_tracking_node, rviz_node, mediapipe_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_trajectory",  default_value="true",
                              description="true  -> JTC  (arm_controller). "
                                          "false -> JGPC (arm_group_position_controller). "
                                          "Must match nxp_omniman_launch.py use_trajectory."),
        DeclareLaunchArgument("show_window",     default_value="true"),
        DeclareLaunchArgument("start_mediapipe", default_value="true"),
        DeclareLaunchArgument("start_rviz",      default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])