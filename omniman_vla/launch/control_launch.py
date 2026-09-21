"""
The control lock and the policy runner.

    control_arbiter  /control/owner, /control/acquire, /control/release
    policy_runner    /policy_runner/run, /policy_runner/stop, /policy_runner/status
    visual_align     /visual_align/run, /visual_align/stop, /visual_align/status
    color_detector   /color_detector/detections, /color_detector/debug/compressed

Anything that wants to take part in the lock - pick_place_mission.py, the web
UI's Policy switch - needs these two running. Programs that never acquire
control are unaffected.

Run:
  ros2 launch omniman_vla control_launch.py

Also started by nxp_omniman_launch.py, so usually nothing to run by hand.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    runner_params = PathJoinSubstitution(
        [FindPackageShare('omniman_vla'), 'config', 'policy_runner.yaml'])

    control_arbiter = Node(
        package='omniman_vla',
        executable='control_arbiter.py',
        name='control_arbiter',
        output='screen',
    )

    policy_runner = Node(
        package='omniman_vla',
        executable='policy_runner.py',
        name='policy_runner',
        output='screen',
        parameters=[runner_params],
    )

    # Aligns to what color_detector finds; idle until ~/run is called. It and
    # color_detector read config/visual_align.yaml themselves - no parameters.
    visual_align = Node(
        package='omniman_vla',
        executable='visual_align.py',
        name='visual_align',
        output='screen',
    )

    # Finds the align targets by colour; plain OpenCV, no GPU. Idle until
    # visual_align (or the debug image) subscribes.
    color_detector = Node(
        package='omniman_vla',
        executable='color_detector.py',
        name='color_detector',
        output='screen',
    )

    return LaunchDescription([control_arbiter, policy_runner, visual_align, color_detector])
