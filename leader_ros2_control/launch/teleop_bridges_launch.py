#!/usr/bin/env python3
#
# Leader -> follower teleop bridges. This launch is the deliberate "activation
# button" for leader-follower teleoperation: it starts the arm relay and the
# gripper action bridge that make omniman track the leader.
#
# Workflow:
#   1. bring up the follower (omniman) and home it
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
    # Arm: relay the leader's 6-joint JointTrajectory straight onto the
    # follower's arm_controller (joint names already match omniman exactly).
    arm_trajectory_relay = Node(
        package='topic_tools',
        executable='relay',
        name='arm_trajectory_relay',
        arguments=['/leader/joint_trajectory', '/arm_controller/joint_trajectory'],
        output='screen',
    )

    # Gripper: bridge the leader trigger position -> follower GripperCommand
    # action (different joint name, units, and controller type, so it can't be a
    # plain relay).
    gripper_teleop_bridge = Node(
        package='leader_ros2_control',
        executable='gripper_teleop_bridge.py',
        name='gripper_teleop_bridge',
        output='screen',
    )

    return LaunchDescription([
        arm_trajectory_relay,
        gripper_teleop_bridge,
    ])
