"""
Everything that coordinates the robot - run it on the PC with the GPU and
physical_ai_server, NOT on the robot. The robot launch (nxp_omniman_launch.py)
only brings up hardware: controllers, camera, lidar, joystick.

    control_arbiter  /control/owner, /control/acquire, /control/release
    policy_runner    /policy_runner/run, /policy_runner/stop, /policy_runner/status
    detector         /<detector>/detections, /<detector>/debug/compressed
    visual_align     /visual_align/run, /visual_align/stop, /visual_align/status
    grasp_monitor    /gripper/holding, /grasp_monitor/state

Anything that takes part in the lock - the missions, the web UI's
Policy and Align switches - needs this running; without it they report
"control_arbiter not answering" and nothing moves. The joystick is not part of
the lock and always works.

The detector and visual_align read config/visual_align.yaml on THIS PC and
pick up saved edits within a second - no restart, nothing to change on the
robot. Run only one copy of this launch on the network.

DETECTOR - one of two, switched by hand below (comment one in, the other out):
  sam_detector            SAM 3
  efficient_sam_detector  EfficientSAM3
Both need the omniman_vla conda env (omniman_vla/requirements.txt,
docs/omniman_vla.md "Setup").
Each publishes under its own name (/sam_detector/... or
/efficient_sam_detector/...): when switching, also set visual_align's
detections_topic in config/visual_align.yaml, and point the debug viewer at
the matching debug/compressed.

Run:
  conda activate omniman_vla && source install/setup.bash
  ros2 launch omniman_vla control_launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Nodes log warnings and errors only: what they are doing is on their topics,
# and the mission (pick_place_bt.py) prints it where it belongs, next to the
# step that is waiting for it. Drop this from a node to hear it again.
QUIET = ['--ros-args', '--log-level', 'warn']


def generate_launch_description():
    runner_params = PathJoinSubstitution(
        [FindPackageShare('omniman_vla'), 'config', 'policy_runner.yaml'])

    control_arbiter = Node(
        package='omniman_vla',
        executable='control_arbiter.py',
        name='control_arbiter',
        output='screen',
        arguments=QUIET,
    )

    policy_runner = Node(
        package='omniman_vla',
        executable='policy_runner.py',
        name='policy_runner',
        output='screen',
        parameters=[runner_params],
        arguments=QUIET,
    )

    # SAM 3.
    # detector = Node(
    #     package='omniman_vla',
    #     executable='sam_detector.py',
    #     name='sam_detector',
    #     output='screen',
    #     arguments=QUIET,
    # )

    # EfficientSAM3 - lighter. Publishes on
    # /efficient_sam_detector/... - set visual_align's detections_topic to match.
    detector = Node(
        package='omniman_vla',
        executable='efficient_sam_detector.py',
        name='efficient_sam_detector',
        output='screen',
        arguments=QUIET,
    )

    # Aligns to the first thing the detector finds; idle until ~/run.
    visual_align = Node(
        package='omniman_vla',
        executable='visual_align.py',
        name='visual_align',
        output='screen',
        arguments=QUIET,
    )

    # Is the gripper holding something? From the gripper joint's position and
    # effort (settings in config/mission.yaml); pick_place_bt.py retries a
    # pick that closed on nothing.
    grasp_monitor = Node(
        package='omniman_vla',
        executable='grasp_monitor.py',
        name='grasp_monitor',
        output='screen',
        arguments=QUIET,
    )

    return LaunchDescription(
        [control_arbiter, policy_runner, detector, visual_align, grasp_monitor])
