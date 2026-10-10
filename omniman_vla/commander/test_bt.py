#!/usr/bin/env python3
"""Test mission: arm to ready1, find the cup from the arm poses and align (twice),
then run the pick policy. No driving: stand the robot at the pick area first.

Run:
  ros2 run omniman_vla test_bt.py
"""

from omniman_vla.mission import ArmPose, PolicyStep, run_mission, search_align, task

CUP = 'yellow cup lid'


def build(robot):
    policy = robot.cfg['policies']['manipulate']['path']
    return task('post arrival', steps=[
        ArmPose(robot, 'ready1'),
        search_align(robot, CUP, timeout_s=60),
        search_align(robot, CUP, timeout_s=60),
        PolicyStep(robot, 'pick', 'pick the object', policy_path=policy, timeout_s=90),
    ])


if __name__ == '__main__':
    run_mission(build, node_name='test_bt')
