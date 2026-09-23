#!/usr/bin/env python3
"""
Pick and place as a behaviour tree - the worked example of a mission built
from omniman_vla.mission (docs/omniman_vla.md, part 3).

    pick and place                        Sequence
     ├─ fetch attempts                    Retry (FETCH_ATTEMPTS)
     │    ├─ nav to pick_area             Navigate
     │    ├─ pick attempts                Retry (PICK_ATTEMPTS)
     │    │    └─ attempt                 Sequence
     │    │         ├─ align pick_target  Align        (align_before_pick)
     │    │         ├─ pick policy        PolicyStep
     │    │         └─ grasp succeeded    Holding(True)
     │    └─ while holding                EternalGuard (holding)
     │         └─ nav to place_area       Navigate
     ├─ place attempts                    Retry (PLACE_ATTEMPTS)
     │    └─ attempt                      Sequence
     │         ├─ align place_target      Align        (align_before_place)
     │         ├─ place policy            PolicyStep
     │         └─ cup released            Holding(False)
     └─ nav to home                       Navigate

The pick policy closes the gripper whether or not it got the cup;
grasp_monitor tells the two apart, and a failed grasp makes Retry align and
pick again instead of driving off empty. Losing the cup on the way (dropped,
or taken out of the gripper) fails the guard, which cancels the drive; the
fetch then starts over - back to the pick area, base correction, pick again.
The place is the mirror: still holding after the place policy means it did
not let go - align to the mark again and retry. This is the standard pick-with-retry pattern
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
from omniman_vla.mission import (Align, Holding, Navigate, PolicyStep, attempts,
                                 run_mission)


# How many tries each part gets, and how long its steps may take. They live
# here, not in mission.yaml: another mission wants other numbers. Leaving a
# timing out uses the mission file's setting (settle_timeout_s, service_wait_s).
FETCH_ATTEMPTS = 3          # cup lost on the way -> back to the pick area
PICK_ATTEMPTS = 3           # gripper closed on nothing -> align and pick again
PLACE_ATTEMPTS = 3          # still holding the cup -> align and place again
ALIGN_TIMEOUT_S = 60.0      # give up on visual_align (it searches, then fails)
POLICY_TIMEOUT_S = 90.0     # stop a policy that neither finishes nor gives up


def build(robot):
    s = robot.cfg['settings']
    m = robot.cfg['policies']['manipulate']

    # What the robot knows is on the robot itself -
    # robot.is_holding(), .gripper_state(), .arm_state(), .align_status(),
    # .policy_status(), .task_phase(), .control_owner(), .base_pose() - for
    # conditions like the guard below, and for missions that log their own.
    pick = ([Align(robot, s['pick_target'], timeout_s=ALIGN_TIMEOUT_S)]
            if s.get('align_before_pick', True) else [])
    pick += [PolicyStep(robot, 'pick', m['instruction_pick'], policy_path=m['path'],
                        timeout_s=POLICY_TIMEOUT_S),
             Holding(robot, 'grasp succeeded', holding=True)]

    place = ([Align(robot, s['place_target'], timeout_s=ALIGN_TIMEOUT_S)]
             if s.get('align_before_place', False) else [])
    place += [PolicyStep(robot, 'place', m['instruction_place'], policy_path=m['path'],
                         timeout_s=POLICY_TIMEOUT_S),
              Holding(robot, 'cup released', holding=False)]

    # Fetching is one unit: drive to the pick area, pick, carry. Losing the
    # cup on the way (dropped, or taken out of the gripper) fails the guard,
    # which cancels the drive - and the whole fetch starts again: back to the
    # pick area, base correction (searching for the cup if it is out of view),
    # pick. Only after FETCH_ATTEMPTS does the mission give up.
    fetch = [Navigate(robot, 'pick_area'),
             attempts('pick attempts', pick, PICK_ATTEMPTS),
             py_trees.decorators.EternalGuard(
                 'while holding', Navigate(robot, 'place_area'),
                 condition=robot.is_holding)]

    return py_trees.composites.Sequence('pick and place', memory=True, children=[
        attempts('fetch attempts', fetch, FETCH_ATTEMPTS),
        attempts('place attempts', place, PLACE_ATTEMPTS),
        Navigate(robot, 'home'),
    ])


if __name__ == '__main__':
    run_mission(build, node_name='pick_place_bt')
