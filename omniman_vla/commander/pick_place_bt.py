#!/usr/bin/env python3
"""
Pick and place as a behaviour tree - the worked example of a mission built
from omniman_vla.mission (docs/omniman_vla.md, part 3).

    pick and place                       Sequence
     ├─ nav to pick_area                 Navigate
     ├─ pick attempts                    Retry (max_pick_attempts)
     │    └─ attempt                     Sequence
     │         ├─ align to pick_target   Align        (align_before_pick)
     │         ├─ pick policy            PolicyStep
     │         └─ grasp succeeded        Holding(True)
     ├─ while holding                    EternalGuard (holding)
     │    └─ nav to place_area           Navigate
     ├─ place attempts                   Retry (max_place_attempts)
     │    └─ attempt                     Sequence
     │         ├─ align to place_target  Align        (align_before_place)
     │         ├─ place policy           PolicyStep
     │         └─ cup released           Holding(False)
     └─ nav to home                      Navigate

The pick policy closes the gripper whether or not it got the cup;
grasp_monitor tells the two apart, and a failed grasp makes Retry align and
pick again instead of driving off empty. If the cup drops on the way, the
guard cancels the drive and the mission stops. The place is the mirror: still
holding after the place policy means it did not let go - align to the mark
again and retry. This is the standard pick-with-retry pattern
(BehaviorTree.CPP's RetryUntilSuccessful; py_trees' Retry and EternalGuard)
with QT-Opt's style of grasp check (gripper not fully closed).

Settings from config/mission.yaml (`settings:`, `policies: manipulate`),
places from poses.yaml. pick_target / place_target must be prompts of the
running detector - checked at start.

Prereqs: nav2_launch.py (robot localized), physical_ai_server,
control_launch.py; py_trees (sudo apt install ros-jazzy-py-trees).

Run:
  ros2 run omniman_vla pick_place_bt.py
"""

import py_trees
from omniman_vla.mission import Align, Holding, Navigate, PolicyStep, run_mission


def attempts(name, children, times):
    """Retry `children` (in order) up to `times` failures."""
    return py_trees.decorators.Retry(
        name, py_trees.composites.Sequence('attempt', memory=True, children=children),
        num_failures=int(times))


def build(robot):
    s = robot.cfg['settings']
    m = robot.cfg['policies']['manipulate']

    pick = [Align(robot, s['pick_target'])] if s.get('align_before_pick', True) else []
    pick += [PolicyStep(robot, 'pick', m['instruction_pick'], policy_path=m['path']),
             Holding(robot, 'grasp succeeded', holding=True)]

    place = [Align(robot, s['place_target'])] if s.get('align_before_place', False) else []
    place += [PolicyStep(robot, 'place', m['instruction_place'], policy_path=m['path']),
              Holding(robot, 'cup released', holding=False)]

    return py_trees.composites.Sequence('pick and place', memory=True, children=[
        Navigate(robot, 'pick_area'),
        attempts('pick attempts', pick, s['max_pick_attempts']),
        py_trees.decorators.EternalGuard(
            'while holding', Navigate(robot, 'place_area'), condition=lambda: robot.holding),
        attempts('place attempts', place, s['max_place_attempts']),
        Navigate(robot, 'home'),
    ])


if __name__ == '__main__':
    run_mission(build, node_name='pick_place_bt')
