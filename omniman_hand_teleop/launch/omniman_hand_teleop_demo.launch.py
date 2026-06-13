"""
Omniman single-arm hand teleop -- SIMULATION / RViz visualization.

This is the "try before real hardware" launch. It brings up EVERYTHING with
fake hardware (mock_components/GenericSystem from the moveit_config URDF):

  * robot_state_publisher
  * ros2_control_node (fake hardware) + joint_state_broadcaster
  * arm_controller (JointTrajectoryController) + gripper_controller (GripperAction)
  * omniman_pose_tracking_node  (MoveIt Servo pose tracker)
  * RViz showing the robot model and the /target_pose arrow marker
  * hand_pose_publisher_node    (MediaPipe right-hand tracker) -- optional

You should see the simulated arm follow your right hand, and a red arrow in
RViz showing the commanded target end-effector pose. No move_group is started
(the servo node owns the planning scene).

Args:
  camera_device      webcam index (default 2 -> /dev/video2)
  track_orientation  follow hand orientation too (default false = position-only)
  show_window        OpenCV camera overlay (default true)
  start_mediapipe    start the Python node here (default true).
                     Set false to drive /hand_target_pose manually with
                     `ros2 topic pub` for a first dry run.
  start_rviz         start RViz (default true)
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
    track_orientation = LaunchConfiguration("track_orientation").perform(context).lower() == "true"
    show_window = LaunchConfiguration("show_window").perform(context).lower() == "true"
    use_fixed_orientation = LaunchConfiguration("use_fixed_orientation").perform(context).lower() == "true"
    fixed_roll = float(LaunchConfiguration("fixed_roll").perform(context))
    fixed_pitch = float(LaunchConfiguration("fixed_pitch").perform(context))
    fixed_yaw = float(LaunchConfiguration("fixed_yaw").perform(context))

    moveit_config = (
        MoveItConfigsBuilder("nxp_omniman", package_name="moveit_config")
        .robot_description(file_path="config/nxp_omniman.urdf.xacro")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("omniman_hand_teleop")
    teleop_params = os.path.join(pkg_share, "config", "teleop.yaml")
    pt_dir = os.path.join(pkg_share, "config", "pose_tracking")
    servo_params = {
        "moveit_servo": ParameterBuilder("omniman_hand_teleop")
        .yaml(os.path.join(pt_dir, "pose_tracking_settings.yaml"))
        .yaml(os.path.join(pt_dir, "omniman_pose_tracking.yaml"))
        .to_dict()
    }

    # Sim always uses arm_controller (JTC); override the yaml default which
    # points at arm_group_position_controller (JGPC, not spawned in sim).
    s = servo_params["moveit_servo"]
    s["command_out_type"]         = "trajectory_msgs/JointTrajectory"
    s["command_out_topic"]        = "/arm_controller/joint_trajectory"
    s["publish_joint_velocities"] = True

    # --- Robot state publisher ------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # --- ros2_control (fake hardware) + controllers ---------------------------
    # Sim-only controllers: same as moveit_config but joint_state_broadcaster
    # also publishes the passive mecanum wheels so RViz has wheel TF.
    ros2_controllers_path = os.path.join(pkg_share, "config", "sim", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager", "--controller-manager-timeout", "60"],
            output="screen",
        )

    jsb_spawner = spawner("joint_state_broadcaster")
    arm_spawner = spawner("arm_controller")
    gripper_spawner = spawner("gripper_controller")

    # --- Servo pose tracking node ---------------------------------------------
    pose_tracking_node = Node(
        package="omniman_hand_teleop",
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

    # --- RViz -----------------------------------------------------------------
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

    # --- MediaPipe (optional) -------------------------------------------------
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

    return [
        robot_state_publisher,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        gripper_spawner,
        pose_tracking_node,
        rviz_node,
        mediapipe_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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