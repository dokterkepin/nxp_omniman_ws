"""
omniman_vla.mission - write a robot mission as a behaviour tree (py_trees).

A mission only draws its tree; run_mission() does the rest:

    import py_trees
    from omniman_vla.mission import Align, Holding, Navigate, PolicyStep, attempts, run_mission

    def build(robot):
        pick = [Align(robot, 'green bottle', timeout_s=60),
                PolicyStep(robot, 'pick', 'pick the bottle', timeout_s=90),
                Holding(robot, 'grasp succeeded', holding=True)]
        return py_trees.composites.Sequence('fetch', memory=True, children=[
            Navigate(robot, 'shelf'),
            attempts('pick attempts', pick, 3),
            Navigate(robot, 'home'),
        ])

    if __name__ == '__main__':
        run_mission(build, node_name='fetch_mission')

commander/pick_place_bt.py is the full example.

Steps (steps.py)
    Navigate(robot, place)                  drive with Nav2 to a place in poses.yaml
    Align(robot, target)                    base correction with visual_align;
                                            target = any text, the detector's prompt
    ArmPose(robot, pose)                    move the arm to a pose in arm_poses.yaml; holds
                                            control as "arm" until it has arrived
    together(name, [steps])                 run the steps at the same time (e.g. drive
                                            and move the arm); fails if one fails
    Look(robot, target)                     look for the target from where the arm is,
                                            without turning (detector frames, no time)
    search_align(robot, target, poses)      find the target from each arm pose - first
                                            without turning (Look), then by turning
                                            (Align) - align to it, back to the first pose
    PolicyStep(robot, label, instruction)   run the arm policy until the arm is home;
                                            starts only on a camera frame taken after
                                            the base stopped and the arm arrived
    Holding(robot, name, holding=True)      is the gripper holding something (or not);
                                            waits for grasp_monitor to agree with the
                                            gripper's reading first
    task(name, steps, attempts, on_failure) one state: its steps, how many tries,
                                            and what to run if it still fails -
                                            if those succeed the mission carries on
    start_again()                           last in on_failure: start the mission
                                            again from its first task
    mission(name, [tasks], restarts)        the tasks in order; a failed task
                                            starts the mission again from the first
    attempts(name, [steps], times)          run those steps again if one fails
    on_failure(steps, then=[steps])         if steps fail, run `then` instead -
                                            what happens next is up to the mission

  Timings, per step - left out, the setting of that name in mission.yaml is used:
    timeout_s          Align, PolicyStep   stop it and fail after this long
    settle_timeout_s   Navigate, Align     wait this long for the base to stop
    service_wait_s     all                 wait this long for the service to exist
    move_s             ArmPose             how long the arm move takes (arm_move_s)
  A step that fails stops what it started, so attempts() around it can retry.

  Log: the tree is printed whenever a step changes status; in between, the
  running step says what it is waiting for every log_every_s. Every result and
  failure reason is logged either way.

What the robot knows (robot.py) - for conditions and your own log lines
    robot.is_holding()      grasp_monitor: something in the gripper
    robot.gripper_state()   its reading, "holding (position ..., effort ...)"
    robot.arm_state()       policy_runner: how far the arm is from home
    robot.align_status()    visual_align: searching | aligning | aligned | failed
    robot.policy_status()   policy_runner: idle | starting | working
    robot.task_phase()      Cyclo orchestrator: LOADING | INFERENCING | PAUSED | READY
    robot.control_owner()   the control lock: nav | arm | align | policy | ""
    robot.base_pose()       (x, y, yaw) from odometry
    robot.base_twist()      (vx, vy, wz) from odometry
    robot.base_still()      the latest odometry is below still_linear / still_angular
  e.g. py_trees.decorators.EternalGuard('while holding', step, condition=robot.is_holding)

run_mission(build, node_name) (runner.py)
    ROS setup, mission.yaml and poses.yaml, the initial pose, waiting for Nav2, the tick loop
    and the log - and on any exit, Ctrl+C included, it stops Nav2,
    visual_align and the policy and releases the control lock.

A new kind of step: subclass Step (step.py) - see Navigate for the pattern.

Files
    __init__.py   this guide and the public names
    steps.py      the steps
    step.py       Step, the base class the steps share
    robot.py      Robot: topics, services, what they last said
    runner.py     run_mission()

Settings: mission.yaml `settings:`; places: poses.yaml beside it; arm poses:
arm_poses.yaml beside it.
Needs py_trees: sudo apt install ros-jazzy-py-trees
"""

from .robot import Robot
from .runner import run_mission
from .step import Step
from .steps import (Align, ArmPose, Holding, Look, Navigate, PolicyStep, attempts, mission,
                    on_failure, search_align, start_again, task, together)

__all__ = ['Align', 'ArmPose', 'Holding', 'Look', 'Navigate', 'PolicyStep', 'Robot', 'Step',
           'attempts', 'mission', 'on_failure', 'run_mission', 'search_align', 'start_again',
           'task', 'together']
