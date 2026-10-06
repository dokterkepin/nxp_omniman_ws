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
    OmnimanGraspCheck   grasp_monitor: is the gripper holding (or not)
    StartAgain          always fails - last in an OnFailure recovery, to make an
                        enclosing Attempts run everything again

The engine stops a tree by resetting every node, so a node that started
something - a Nav2 goal, visual_align, a policy - cancels it in reset().
"""

from orchestrator.bt.actions.omniman.align import OmnimanAlign
from orchestrator.bt.actions.omniman.arm_home import OmnimanArmHome
from orchestrator.bt.actions.omniman.grasp_check import OmnimanGraspCheck
from orchestrator.bt.actions.omniman.navigate import OmnimanNavigate
from orchestrator.bt.actions.omniman.policy import OmnimanPolicy
from orchestrator.bt.actions.omniman.start_again import StartAgain

NODES = (OmnimanNavigate, OmnimanAlign, OmnimanPolicy, OmnimanArmHome,
         OmnimanGraspCheck, StartAgain)

# Cyclo's node registry lists the classes defined directly in a module of
# actions/. Ours are one level down, so they say they live here.
for _node in NODES:
    _node.__module__ = __name__
