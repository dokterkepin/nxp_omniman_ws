#!/usr/bin/env python3
"""
omniman's steps as nodes for Cyclo's behaviour-tree engine (Autonomy Studio,
Action Canvas). One file per node; each class is one node in the Studio
palette and its constructor arguments are the fields the UI shows:

    OmnimanNavigate     Nav2 to a place in omniman_vla/config/poses.yaml,
                        holding the control lock as "nav" while driving
    OmnimanAlign        base correction: visual_align on what the SAM detector
                        finds for `target` (any text)
    OmnimanPolicy       an arm policy through policy_runner, until the arm is home
    OmnimanArmHome      waits until the arm is home for dwell_s: the end of a
                        run started with Cyclo's own SendCommand
    OmnimanGraspCheck   grasp_monitor: is the gripper holding (or not); waits
                        until it agrees with the gripper joint
    OmnimanArmPose      moves the arm to a named pose in arm_poses.yaml and waits
                        until the arm controller says it has arrived
    OmnimanLook         waits for fresh camera frames, then asks whether the
                        SAM detector sees `target`
    OmnimanSearchAlign  looks for `target` from the arm's ready poses (without
                        turning, then turning), then aligns to it
    OmnimanWaitSettled  waits until the base is still, the arm has finished and
                        a camera frame has arrived after both
    StartAgain          always fails - last in an OnFailure recovery, to make an
                        enclosing Attempts run everything again

The engine stops a tree by resetting every node, so a node that started
something - a Nav2 goal, visual_align, a policy - cancels it in reset().
"""

from orchestrator.bt.actions.omniman.align import OmnimanAlign
from orchestrator.bt.actions.omniman.arm_home import OmnimanArmHome
from orchestrator.bt.actions.omniman.arm_pose import OmnimanArmPose
from orchestrator.bt.actions.omniman.grasp_check import OmnimanGraspCheck
from orchestrator.bt.actions.omniman.look import OmnimanLook
from orchestrator.bt.actions.omniman.navigate import OmnimanNavigate
from orchestrator.bt.actions.omniman.policy import OmnimanPolicy
from orchestrator.bt.actions.omniman.search_align import OmnimanSearchAlign
from orchestrator.bt.actions.omniman.start_again import StartAgain
from orchestrator.bt.actions.omniman.wait_settled import OmnimanWaitSettled

NODES = (OmnimanNavigate, OmnimanAlign, OmnimanPolicy, OmnimanArmHome,
         OmnimanGraspCheck, OmnimanArmPose, OmnimanLook, OmnimanSearchAlign,
         OmnimanWaitSettled, StartAgain)

# Cyclo's node registry lists the classes defined directly in a module of
# actions/. Ours are one level down, so they say they live here.
for _node in NODES:
    _node.__module__ = __name__
