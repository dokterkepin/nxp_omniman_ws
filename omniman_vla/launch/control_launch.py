"""
Everything that coordinates the robot - run it on the PC with the GPU and
physical_ai_server, NOT on the robot. The robot launch (nxp_omniman_launch.py)
only brings up hardware: controllers, camera, lidar, joystick.

    control_arbiter  /control/owner, /control/acquire, /control/release
    policy_runner    /policy_runner/run, /policy_runner/stop, /policy_runner/status
    detector         /<detector>/detections, /<detector>/debug/compressed
    visual_align     /visual_align/run, /visual_align/stop, /visual_align/status

Anything that takes part in the lock - pick_place_mission.py, the web UI's
Policy and Align switches - needs this running; without it they report
"control_arbiter not answering" and nothing moves. The joystick is not part of
the lock and always works.

The detector and visual_align read config/visual_align.yaml on THIS PC and
pick up saved edits within a second - no restart, nothing to change on the
robot. Run only one copy of this launch on the network.

DETECTOR - one of two, switched by hand below (comment one in, the other out):
  sam_detector            SAM 3            launch from the lerobot_jazzy env
  efficient_sam_detector  EfficientSAM3    launch from the effsam3 env
Each publishes under its own name (/sam_detector/... or
/efficient_sam_detector/...): when switching, also set visual_align's
detections_topic in config/visual_align.yaml, and point the debug viewer at
the matching debug/compressed.

Run (with the env of the detector that is switched in):
  conda activate effsam3 && source install/setup.bash    # or lerobot_jazzy
  ros2 launch omniman_vla control_launch.py
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

    # SAM 3 - launch from the lerobot_jazzy env.
    # detector = Node(
    #     package='omniman_vla',
    #     executable='sam_detector.py',
    #     name='sam_detector',
    #     output='screen',
    # )

    # EfficientSAM3 - lighter; launch from the effsam3 env. Publishes on
    # /efficient_sam_detector/... - set visual_align's detections_topic to match.
    detector = Node(
        package='omniman_vla',
        executable='efficient_sam_detector.py',
        name='efficient_sam_detector',
        output='screen',
    )

    # Aligns to the first thing the detector finds; idle until ~/run.
    visual_align = Node(
        package='omniman_vla',
        executable='visual_align.py',
        name='visual_align',
        output='screen',
    )

    return LaunchDescription([control_arbiter, policy_runner, detector, visual_align])
