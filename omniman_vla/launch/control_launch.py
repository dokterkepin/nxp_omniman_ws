"""
The control lock and the policy runner.

    control_arbiter  /control/owner, /control/acquire, /control/release
    policy_runner    /policy_runner/run, /policy_runner/stop, /policy_runner/status

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
        [FindPackageShare('omniman_navigation'), 'config', 'policy_runner.yaml'])

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

    return LaunchDescription([control_arbiter, policy_runner])
