from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare('omniman_vla')

    robot_controllers = PathJoinSubstitution(
        [pkg_path, 'config', 'controllers_vla.yaml']
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [pkg_path, 'description', 'nxp_omniman.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # 7-joint arm_controller (6 arm + gripper_prismatic_joint)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', robot_controllers],
    )

    # arm_controller waits on joint_state_broadcaster.
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )


    # Wrist camera (mounted on palm_link, moves with the arm).
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_wrist',
        parameters=[{
            'video_device': '/dev/video_c930',
            'pixel_format': 'mjpeg2rgb',
            'focus_auto': 0,
            'focus_absolute': 30,
            'brightness': 128,
        }],
    )

    # Workspace (fixed top-down) camera. Lives here rather than in
    workspace_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_workspace',
        parameters=[{
            'video_device': '/dev/video_c920',
            'pixel_format': 'mjpeg2rgb',
        }],
        namespace='cam_workspace',
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller,
            usb_cam,
            workspace_cam,
        ]
    )
