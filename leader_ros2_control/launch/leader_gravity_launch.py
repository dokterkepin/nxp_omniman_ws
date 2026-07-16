from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/dynamixel_leader',
            description='Serial port for the leader dynamixel chain.',
        ),
    ]

    port_name = LaunchConfiguration('port_name')

    pkg_path = FindPackageShare('leader_ros2_control')

    robot_controllers = PathJoinSubstitution(
        [pkg_path, 'config', 'controllers_gravity.yaml']
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [pkg_path, 'description', 'leader.gravity.xacro']
            ),
            ' ',
            'port_name:=',
            port_name,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    # frame_prefix avoids TF collision with the follower (omniman) links
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'gravity_compensation_controller',
            'spring_actuator_controller',
            'joint_trajectory_command_broadcaster',
        ],
        parameters=[robot_description],
    )

    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_state_publisher_node,
            controller_spawner,
        ]
    )

    return LaunchDescription(declared_arguments + [leader_with_namespace])
