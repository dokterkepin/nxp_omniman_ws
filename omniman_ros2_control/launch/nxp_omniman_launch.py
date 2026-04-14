from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_path = FindPackageShare("omniman_ros2_control")

    moveit_config = MoveItConfigsBuilder(
        "nxp_omniman", package_name="omniman_moveit_config"
    ).to_moveit_configs()

    robot_controllers = PathJoinSubstitution(
        [pkg_path, "config", "controllers.yaml"]
    )

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use Isaac Sim mock hardware instead of real motors",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_path, "description", "nxp_omniman.urdf.xacro"]
            ),
            " use_sim:=",
            LaunchConfiguration("use_sim"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    joystick_config = PathJoinSubstitution(
        [pkg_path, "config", "joystick.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/mecanum_drive_controller/reference", "/cmd_vel"),
        ],
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

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", robot_controllers],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--param-file", robot_controllers],
    )

    # Sequential startup: broadcaster -> mecanum + arm -> gripper
    delay_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joystick_config],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[joystick_config],
        remappings=[("/cmd_vel", "/cmd_vel")],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True,
             "publish_robot_description": True},
        ],
    )

    return LaunchDescription(
        [
            use_sim_arg,
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_mecanum_controller,
            delay_arm_controller,
            delay_gripper_controller,
            joy_node,
            teleop_node,
            move_group_node,
        ]
    )
