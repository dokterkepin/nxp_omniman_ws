"""
Omniman single-arm hand teleop (MediaPipe -> MoveIt Servo pose tracking).

Assumes the robot is ALREADY running on real hardware with /joint_states being
published, gripper_controller active, and the arm controller matching the
use_trajectory choice below. NO move_group running (servo owns the planning scene).

Match use_trajectory here to how you brought up omniman_ros2_control:
  use_trajectory:=true  <-> robot use_servo:=false -> arm_controller (JointTrajectoryController)
                            servo publishes trajectory_msgs/JointTrajectory
                            to /arm_controller/joint_trajectory
  use_trajectory:=false <-> robot use_servo:=true  -> arm_group_position_controller (JointGroupPositionController)
                            servo publishes std_msgs/Float64MultiArray
                            to /arm_group_position_controller/commands

This launch therefore starts ONLY:
  * omniman_pose_tracking_node  (C++ MoveIt Servo pose tracker)
  * hand_pose_publisher_node    (Python MediaPipe right-hand tracker)

Launch args:
  camera_device          webcam index for hand tracking (default 2 -> /dev/video2)
  use_trajectory         true -> JTC/arm_controller; false -> JGPC/arm_group_position_controller (default true)
  track_orientation      follow hand orientation too (default false = position-only)
  use_fixed_orientation  hold a specific EE orientation (default false = hold startup pose)
  fixed_roll             RPY roll  [deg] for fixed orientation (default 0)
  fixed_pitch            RPY pitch [deg] for fixed orientation (default 0)
  fixed_yaw              RPY yaw   [deg] for fixed orientation (default 0)
  show_window            show the OpenCV camera overlay (default true)
  start_mediapipe        also start the Python node here (default true)
  start_rviz             open RViz with the /target_pose arrow (default true)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def launch_setup(context, *args, **kwargs):
    camera_device = int(LaunchConfiguration("camera_device").perform(context))
    track_orientation = LaunchConfiguration("track_orientation").perform(context).lower() == "true"
    show_window = LaunchConfiguration("show_window").perform(context).lower() == "true"
    use_fixed_orientation = LaunchConfiguration("use_fixed_orientation").perform(context).lower() == "true"
    fixed_roll = float(LaunchConfiguration("fixed_roll").perform(context))
    fixed_pitch = float(LaunchConfiguration("fixed_pitch").perform(context))
    fixed_yaw = float(LaunchConfiguration("fixed_yaw").perform(context))
    use_trajectory = LaunchConfiguration("use_trajectory").perform(context).lower() == "true"

    # Robot description / SRDF / kinematics come from the omniman moveit_config.
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("mediapipe_dual_arm_control")
    config_dir = os.path.join(pkg_share, "config", "pose_tracking")
    servo_params = {
        "moveit_servo": ParameterBuilder("mediapipe_dual_arm_control")
        .yaml(os.path.join(config_dir, "pose_tracking_settings.yaml"))
        .yaml(os.path.join(config_dir, "omniman_pose_tracking.yaml"))
        .to_dict()
    }

    # Match the servo command output to whichever arm controller ros2_control loaded.
    # Same switch as moveit_servo/launch/servo_example.launch.py (the joystick teleop).
    servo_settings = servo_params["moveit_servo"]
    if use_trajectory:
        servo_settings["command_out_type"] = "trajectory_msgs/JointTrajectory"
        servo_settings["command_out_topic"] = "/arm_controller/joint_trajectory"
        servo_settings["publish_joint_velocities"] = True
    else:
        # JointGroupPositionController takes positions only -> no velocities.
        servo_settings["command_out_type"] = "std_msgs/Float64MultiArray"
        servo_settings["command_out_topic"] = "/arm_group_position_controller/commands"
        servo_settings["publish_joint_velocities"] = False

    pose_tracking_node = Node(
        package="mediapipe_dual_arm_control",
        executable="omniman_pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "track_orientation": track_orientation,
                "use_fixed_orientation": use_fixed_orientation,
                "fixed_roll": fixed_roll,
                "fixed_pitch": fixed_pitch,
                "fixed_yaw": fixed_yaw,
            },
        ],
    )

    # --- RViz (real hardware: shows robot TF + /target_pose arrow) -----------
    # The robot publishes /joint_states and /robot_description on its own.
    # RViz reads those directly -- no extra robot_state_publisher needed here.
    rviz_config = os.path.join(pkg_share, "config", "rviz", "omniman_hand_teleop.rviz")
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
        package="mediapipe_dual_arm_control",
        executable="hand_pose_publisher_node.py",
        name="hand_pose_publisher_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mediapipe")),
        parameters=[
            {"camera_device": camera_device},
            {"show_window": show_window},
            {"planning_frame": "base_link"},
        ],
    )

    return [pose_tracking_node, rviz_node, mediapipe_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="2"),
            DeclareLaunchArgument(
                "use_trajectory", default_value="true",
                description="true -> JointTrajectory to arm_controller (robot use_servo:=false). "
                            "false -> Float64MultiArray to arm_group_position_controller (robot use_servo:=true)."),
            DeclareLaunchArgument("track_orientation", default_value="false"),
            DeclareLaunchArgument("use_fixed_orientation", default_value="false"),
            DeclareLaunchArgument("fixed_roll", default_value="0.0"),
            DeclareLaunchArgument("fixed_pitch", default_value="0.0"),
            DeclareLaunchArgument("fixed_yaw", default_value="0.0"),
            DeclareLaunchArgument("show_window", default_value="true"),
            DeclareLaunchArgument("start_mediapipe", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )