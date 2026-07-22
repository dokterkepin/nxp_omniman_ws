#!/usr/bin/env python3
#
# Leader -> follower teleop bridge. The deliberate "activation button" for
# leader-follower teleoperation: it relays the leader's 7-joint JointTrajectory
# (6 arm + gripper) straight onto the omniman_vla follower's arm_controller.
#
# The gripper is now the 7th joint of that trajectory, already scaled to the
# follower's metres by the broadcaster (scales/offsets in controllers_gravity.yaml),
# so it rides the same relay as the arm -- no separate gripper node, and no
# GripperActionController on the follower. (The old gripper_teleop_bridge.py is
# kept in the package only for the legacy 6-joint follower path.)
#
# Workflow:
#   1. bring up the follower (omniman_vla) and home it
#   2. bring up the leader (leader_gravity_launch.py)
#   3. confirm the two arms are at a safe, roughly-matched pose
#   4. THEN start teleop (this file):
#        ros2 launch leader_ros2_control teleop_bridges_launch.py
#
# The follower starts tracking the leader the instant this launches, so only run
# it once the poses are confirmed safe. Ctrl-C here stops teleop while leaving
# both robots running.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Relay the leader's 7-joint JointTrajectory (6 arm + scaled gripper) onto the
    # follower's arm_controller (joint names match omniman_vla exactly).
    arm_trajectory_relay = Node(
        package='topic_tools',
        executable='relay',
        name='arm_trajectory_relay',
        arguments=['/leader/joint_trajectory', '/arm_controller/joint_trajectory'],
        output='screen',
    )

    return LaunchDescription([
        arm_trajectory_relay,
    ])
