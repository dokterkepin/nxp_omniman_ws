from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare('omniman_vla')

    # Joystick is only for driving the BASE between recording locations -- it is not
    # part of the VLA action space. Set false if driving from another machine.
    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Start joy_linux + teleop_twist_joy for driving the mecanum base.',
    )

    robot_controllers = PathJoinSubstitution(
        [pkg_path, 'config', 'controllers_vla.yaml']
    )

    joystick_config = PathJoinSubstitution(
        [pkg_path, 'config', 'joystick.yaml']
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
        remappings=[
            ('/mecanum_drive_controller/reference', '/cmd_vel_stamped'),
            ('/mecanum_drive_controller/reference_unstamped', '/cmd_vel'),
        ],
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

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
    )

    # 7-joint arm_controller (6 arm + gripper_prismatic_joint)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', robot_controllers],
    )

    # mecanum_drive_controller waits on joint_state_broadcaster.
    delay_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[joystick_config],
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    # No cmd_vel remap: the joystick publishes plain Twist straight onto /cmd_vel,
    # which is now both what the mecanum controller reads (reference_unstamped) and
    # what physical_ai_server records as the base action (leader_mobile:/cmd_vel).
    # Previously remapped to /cmd_vel_stamped, which drove the robot but bypassed
    # /cmd_vel, so base motion never reached the dataset.
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[joystick_config],
        condition=IfCondition(LaunchConfiguration('use_joy')),
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
    # workspace_cam = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='usb_cam_workspace',
    #     parameters=[{
    #         'video_device': '/dev/video_c920',
    #         'pixel_format': 'mjpeg2rgb',
    #     }],
    #     namespace='cam_workspace',
    # )

    rplidar_node = Node(  # noqa: F841
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 256000,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        output="screen",
    )

    teleop_inference_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('leader_ros2_control'), 'launch', 'teleop_bridges_launch.py']
            )
        ),
    )

    return LaunchDescription(
        [
            use_joy_arg,
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_mecanum_controller,
            delay_arm_controller,
            joy_node,
            teleop_joy_node,
            usb_cam,
            # rplidar_node,
            # workspace_cam,
            teleop_inference_launch,
        ]
    )
