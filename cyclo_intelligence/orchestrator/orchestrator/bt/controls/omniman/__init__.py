#!/usr/bin/env python3
"""
Controls that give Cyclo's behaviour trees retries and recovery - the same
shapes as omniman_vla.mission's task(), attempts() and mission() - plus the
carry guard and the control lock. One file per control:

    Attempts        children in order; if one fails, all of them again from
                    the first, up to max_attempts tries
    OnFailure       the first child is the work; if it fails, the other
                    children run in order - the recovery. Recovery succeeds:
                    the mission carries on. Recovery fails (or ends with
                    StartAgain): OnFailure fails, so an Attempts around the
                    mission starts it again
    WhileHolding    children in order while grasp_monitor says the gripper is
                    holding; the moment it is not, they are stopped (a Nav2
                    goal cancelled) and it fails
    WithControlLock children in order holding the control lock as `owner`
                    (e.g. "policy" around Cyclo's SendCommand RESUME, so Nav2
                    and visual_align cannot move the robot meanwhile)

A pick-and-place as in pick_place_bt.py:

    Attempts max_attempts=3                          (the mission: restarts)
      OmnimanNavigate place=pick_area
      OnFailure                                      (task "pick")
        Attempts max_attempts=3
          OmnimanAlign target="yellow cup lid"
          OmnimanPolicy instruction="pick the object"
          OmnimanGraspCheck holding=true
        OmnimanNavigate place=home
        StartAgain
      WhileHolding                                   (task "carry")
        OmnimanNavigate place=place_area
      ...
"""

from orchestrator.bt.controls.omniman.attempts import Attempts
from orchestrator.bt.controls.omniman.on_failure import OnFailure
from orchestrator.bt.controls.omniman.while_holding import WhileHolding
from orchestrator.bt.controls.omniman.with_control_lock import WithControlLock

CONTROLS = (Attempts, OnFailure, WhileHolding, WithControlLock)

# Cyclo's node registry lists the classes defined directly in a module of
# controls/. Ours are one level down, so they say they live here.
for _control in CONTROLS:
    _control.__module__ = __name__
