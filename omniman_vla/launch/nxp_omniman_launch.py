import os

from ament_index_python.packages import get_package_share_directory
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
                [pkg_path, 'urdf', 'omniman.urdf.xacro']
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

    # ---- Camera (full frame rate, rectified) ----
    calibration_url = 'file://' + os.path.join(
        get_package_share_directory('omniman_vla'),
        'config', 'camera_calibration.yaml')

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[{
            'video_device': '/dev/video_c930',
            'camera_name': 'narrow_stereo',
            'focus_auto': 0,
            'focus_absolute': 30,
            'frame_id': 'camera_optical_frame',
            'camera_info_url': calibration_url,
            'pixel_format': 'yuyv2rgb',
        }],
    )

    # Rectify at full rate straight from /image_raw (no throttling — VLA wants every
    # frame). image_proc republishes via image_transport, so /image_rect/compressed
    # (the topic physical_ai_server records) is produced automatically.
    image_rectify = Node(
        package='image_proc',
        executable='rectify_node',
        ros_arguments=['--log-level', 'ERROR'],
        remappings=[
            ('image', '/image_raw'),
            ('camera_info', '/camera_info'),
            ('image_rect', '/image_rect'),
        ],
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller,
            usb_cam,
            image_rectify,
        ]
    )
