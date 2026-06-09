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
    use_trajectory       = LaunchConfiguration("use_trajectory").perform(context).lower() == "true"
    track_orientation    = LaunchConfiguration("track_orientation").perform(context).lower() == "true"
    use_fixed_orientation = LaunchConfiguration("use_fixed_orientation").perform(context).lower() == "true"
    fixed_roll           = float(LaunchConfiguration("fixed_roll").perform(context))
    fixed_pitch          = float(LaunchConfiguration("fixed_pitch").perform(context))
    fixed_yaw            = float(LaunchConfiguration("fixed_yaw").perform(context))
    camera_device        = int(LaunchConfiguration("camera_device").perform(context))
    show_window          = LaunchConfiguration("show_window").perform(context).lower() == "true"
    workspace_angle      = float(LaunchConfiguration("workspace_angle").perform(context))

    pkg_share  = get_package_share_directory("mediapipe_dual_arm_control")
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
        package="mediapipe_dual_arm_control",
        executable="omniman_pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "track_orientation":     track_orientation,
                "use_fixed_orientation": use_fixed_orientation,
                "fixed_roll":            fixed_roll,
                "fixed_pitch":           fixed_pitch,
                "fixed_yaw":             fixed_yaw,
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
            "camera_device":   camera_device,
            "show_window":     show_window,
            "planning_frame":  "base_link",
            "workspace_angle": workspace_angle,
        }],
    )

    return [pose_tracking_node, rviz_node, mediapipe_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_device",   default_value="0",
                              description="Webcam index (0 -> /dev/video0, C920)"),
        DeclareLaunchArgument("use_trajectory",  default_value="false",
                              description="false -> JGPC (arm_group_position_controller). "
                                          "true  -> JTC  (arm_controller). "
                                          "Must match nxp_omniman_launch.py use_trajectory."),
        DeclareLaunchArgument("track_orientation",     default_value="false"),
        DeclareLaunchArgument("use_fixed_orientation", default_value="false"),
        DeclareLaunchArgument("fixed_roll",   default_value="0.0",  description="EE roll [deg]"),
        DeclareLaunchArgument("fixed_pitch",  default_value="0.0",  description="EE pitch [deg]"),
        DeclareLaunchArgument("fixed_yaw",    default_value="0.0",  description="EE yaw [deg]"),
        DeclareLaunchArgument("workspace_angle", default_value="0.0",
                              description="Zone angle [deg]: 0=front, 90=left, -90=right, 180=back"),
        DeclareLaunchArgument("show_window",     default_value="true"),
        DeclareLaunchArgument("start_mediapipe", default_value="true"),
        DeclareLaunchArgument("start_rviz",      default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])