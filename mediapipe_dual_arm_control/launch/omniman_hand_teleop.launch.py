"""
Omniman single-arm hand teleop (MediaPipe -> MoveIt Servo pose tracking).

Assumes the robot is ALREADY running on real hardware:
  * /joint_states is being published
  * arm_controller (JointTrajectoryController) is active and accepting
    /arm_controller/joint_trajectory
  * gripper_controller (GripperActionController) is active
  * NO move_group running (servo owns the planning scene)

This launch therefore starts ONLY:
  * omniman_pose_tracking_node  (C++ MoveIt Servo pose tracker)
  * hand_pose_publisher_node    (Python MediaPipe right-hand tracker)

Launch args:
  camera_device     webcam index for hand tracking (default 2 -> /dev/video2)
  track_orientation follow hand orientation too (default false = position-only)
  show_window       show the OpenCV camera overlay (default true)
  start_mediapipe   also start the Python node here (default true)
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

    # Robot description / SRDF / kinematics come from the omniman moveit_config.
    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    config_dir = os.path.join(
        get_package_share_directory("mediapipe_dual_arm_control"), "config", "pose_tracking"
    )
    servo_params = {
        "moveit_servo": ParameterBuilder("mediapipe_dual_arm_control")
        .yaml(os.path.join(config_dir, "pose_tracking_settings.yaml"))
        .yaml(os.path.join(config_dir, "omniman_pose_tracking.yaml"))
        .to_dict()
    }

    pose_tracking_node = Node(
        package="mediapipe_dual_arm_control",
        executable="omniman_pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {"track_orientation": track_orientation},
        ],
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

    return [pose_tracking_node, mediapipe_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="2",
                                  description="Webcam index for hand tracking (/dev/videoN)."),
            DeclareLaunchArgument("track_orientation", default_value="false",
                                  description="true = follow hand orientation; false = position-only."),
            DeclareLaunchArgument("show_window", default_value="true",
                                  description="Show the OpenCV camera overlay window."),
            DeclareLaunchArgument("start_mediapipe", default_value="true",
                                  description="Also start the Python MediaPipe node from this launch."),
            OpaqueFunction(function=launch_setup),
        ]
    )