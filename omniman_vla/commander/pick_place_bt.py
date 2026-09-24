#!/usr/bin/env python3
"""
Pick and place as a behaviour tree - the worked example of a mission built
from omniman_vla.mission (docs/omniman_vla.md, part 3).

    pick and place                 mission: a failed task starts it again (restarts 2)
     ├─ go to pick area            task: nav to pick_area           | on failure: nav home
     ├─ pick                       task (3 tries): align, pick,
     │                                 grasp succeeded              | on failure: nav home
     ├─ carry                      task: nav to place_area while holding
     ├─ place                      task (3 tries): align, place,
     │                                 cup released                 | on failure: nav home
     └─ go home                    task: nav to home

The pick policy closes the gripper whether or not it got the cup;
grasp_monitor tells the two apart, and a failed grasp makes Retry align and
pick again instead of driving off empty. Losing the cup on the way (dropped,
or taken out of the gripper) fails the guard, which cancels the drive; the
fetch then starts over - back to the pick area, base correction, pick again.
The place is the mirror: still holding after the place policy means it did
not let go - align to the mark again and retry. An align that cannot find
its target drives to its area again and aligns once more (on_failure), and a
mission that gives up drives home before it ends. This is the standard
pick-with-retry pattern (BehaviorTree.CPP's RetryUntilSuccessful; py_trees'
Retry, Selector and EternalGuard) with QT-Opt's style of grasp check (gripper
not fully closed).

What to align to and what to tell the policy are written below, in each
task (Align's target is the detector's prompt - any text works). Timing
settings and the policy path from config/mission.yaml, places from poses.yaml.

Prereqs: nav2_launch.py (robot localized), physical_ai_server,
control_launch.py; py_trees (sudo apt install ros-jazzy-py-trees).

Run:
  ros2 run omniman_vla pick_place_bt.py
"""

import py_trees
from omniman_vla.mission import (Align, Holding, Navigate, PolicyStep, mission, run_mission,
                                 start_again, task)


CUP = 'yellow cup lid'           # what to pick - the detector looks for this
MARK = 'black square'            # where to place it


def build(robot):
    policy = robot.cfg['policies']['manipulate']['path']

    # Each task: its steps, how many tries, and what to run if it still
    # fails. If the on_failure steps succeed the mission carries on; end them
    # with start_again() to start the mission again from its first task.
    # on_failure can be any steps - Navigate(robot, '<any place in
    # poses.yaml>'), PolicyStep(robot, 'label', '<any instruction>'), Align...
    go_to_pick = task('go to pick area',
                      steps=[Navigate(robot, 'pick_area')],
                      on_failure=[Navigate(robot, 'home'), start_again()])

    pick = task('pick',
                steps=[Align(robot, CUP, timeout_s=60),
                       PolicyStep(robot, 'pick', 'pick the object',
                                  policy_path=policy, timeout_s=90),
                       Holding(robot, 'grasp succeeded', holding=True)],
                attempts=1,
                # Back to the pick area and straight into the policy - no base
                # correction this time. If this grasp works too, carry on.
                on_failure=[Navigate(robot, 'pick_area'),
                            PolicyStep(robot, 'pick', 'pick the object',
                                       policy_path=policy, timeout_s=90),
                            Holding(robot, 'grasp succeeded', holding=True)])

    carry = task('carry',
                 steps=[py_trees.decorators.EternalGuard(
                     'while holding', Navigate(robot, 'place_area'),
                     condition=robot.is_holding)])

    place = task('place',
                 steps=[Align(robot, MARK, timeout_s=60),
                        PolicyStep(robot, 'place', 'place the object',
                                   policy_path=policy, timeout_s=90),
                        Holding(robot, 'cup released', holding=False)],
                 attempts=3,
                 on_failure=[Navigate(robot, 'home'), start_again()])

    go_home = task('go home', steps=[Navigate(robot, 'home')])

    return mission('pick and place', [go_to_pick, pick, carry, place, go_home], restarts=2)


if __name__ == '__main__':
    run_mission(build, node_name='pick_place_bt')
