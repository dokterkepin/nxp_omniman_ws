import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("mediapipe_dual_arm_control")
    config_dir = os.path.join(pkg_share, "config", "pose_tracking")
    rviz_config = os.path.join(pkg_share, "config", "rviz", "omniman_hand_teleop.rviz")

    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("mediapipe_dual_arm_control")
        .yaml(os.path.join(config_dir, "pose_tracking_settings.yaml"))
        .yaml(os.path.join(config_dir, "omniman_pose_tracking.yaml"))
        .to_dict()
    }

    # --- Launch args ---------------------------------------------------------

    camera_device_arg = DeclareLaunchArgument(
        "camera_device", default_value="0",
        description="Webcam index for MediaPipe (0 -> /dev/video0, HD Pro Webcam C920)")

    track_orientation_arg = DeclareLaunchArgument(
        "track_orientation", default_value="false",
        description="Follow hand orientation (default: position-only)")

    use_fixed_orientation_arg = DeclareLaunchArgument(
        "use_fixed_orientation", default_value="false",
        description="Hold a fixed EE orientation defined by fixed_roll/pitch/yaw")

    fixed_roll_arg  = DeclareLaunchArgument("fixed_roll",  default_value="0.0", description="EE roll [deg]")
    fixed_pitch_arg = DeclareLaunchArgument("fixed_pitch", default_value="0.0", description="EE pitch [deg]")
    fixed_yaw_arg   = DeclareLaunchArgument("fixed_yaw",   default_value="0.0", description="EE yaw [deg]")

    show_window_arg     = DeclareLaunchArgument("show_window",     default_value="true")
    start_mediapipe_arg = DeclareLaunchArgument("start_mediapipe", default_value="true")
    start_rviz_arg      = DeclareLaunchArgument("start_rviz",      default_value="true")

    # --- Nodes ---------------------------------------------------------------

    pose_tracking_node = Node(
        package="mediapipe_dual_arm_control",
        executable="omniman_pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "track_orientation":    ParameterValue(LaunchConfiguration("track_orientation"),    value_type=bool),
                "use_fixed_orientation": ParameterValue(LaunchConfiguration("use_fixed_orientation"), value_type=bool),
                "fixed_roll":  ParameterValue(LaunchConfiguration("fixed_roll"),  value_type=float),
                "fixed_pitch": ParameterValue(LaunchConfiguration("fixed_pitch"), value_type=float),
                "fixed_yaw":   ParameterValue(LaunchConfiguration("fixed_yaw"),   value_type=float),
            },
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
        package="mediapipe_dual_arm_control",
        executable="hand_pose_publisher_node.py",
        name="hand_pose_publisher_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mediapipe")),
        parameters=[{
            "camera_device": LaunchConfiguration("camera_device"),
            "show_window":   LaunchConfiguration("show_window"),
            "planning_frame": "base_link",
        }],
    )

    return LaunchDescription([
        camera_device_arg,
        track_orientation_arg,
        use_fixed_orientation_arg,
        fixed_roll_arg,
        fixed_pitch_arg,
        fixed_yaw_arg,
        show_window_arg,
        start_mediapipe_arg,
        start_rviz_arg,
        pose_tracking_node,
        rviz_node,
        mediapipe_node,
    ])