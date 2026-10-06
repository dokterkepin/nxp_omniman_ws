#!/usr/bin/env python3
"""
Controls that give Cyclo's behaviour trees retries and recovery - the same
shapes as omniman_vla.mission's task(), attempts() and mission() - plus the
carry guard and the control lock:

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

Linked into src/cyclo_intelligence/orchestrator/orchestrator/bt/controls/.
"""

from orchestrator.bt.actions.omniman_actions import omniman
from orchestrator.bt.bt_core import NodeStatus
from orchestrator.bt.controls.base_control import BaseControl
from omniman_interfaces.srv import AcquireControl
import time

RUNNING, SUCCESS, FAILURE = NodeStatus.RUNNING, NodeStatus.SUCCESS, NodeStatus.FAILURE


def _reset_all(children):
    for child in children:
        child.reset()


class Attempts(BaseControl):
    """Children in order, like Sequence. If one fails, every child is reset
    and they run again from the first - up to max_attempts tries in all;
    then it fails."""

    def __init__(self, node, max_attempts: int = 3):
        super().__init__(node, name='Attempts')
        self.max_attempts = max(1, int(max_attempts))
        self.index = 0
        self.failures = 0

    def tick(self):
        if not self.children:
            return FAILURE
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                self.failures += 1
                if self.failures >= self.max_attempts:
                    self.log_warn(f'{child.name} failed - {self.failures} of '
                                  f'{self.max_attempts} tries used, giving up')
                    return FAILURE
                self.log_info(f'{child.name} failed - try {self.failures + 1} of '
                              f'{self.max_attempts}')
                _reset_all(self.children)
                self.index = 0
                return RUNNING
            self.index += 1
        return SUCCESS

    def get_active_node_ids(self):
        if self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self.index = 0
        self.failures = 0


class OnFailure(BaseControl):
    """The first child is the work. If it fails, the other children run in
    order: the recovery - any nodes (drive somewhere, run a policy, ...).
    SUCCESS if the work succeeded, or else if the recovery did; FAILURE if
    the recovery failed too - end it with StartAgain to always make an
    enclosing Attempts start over."""

    def __init__(self, node):
        super().__init__(node, name='OnFailure')
        self.index = 0          # 0 = the work; 1.. = the recovery

    def tick(self):
        if not self.children:
            return FAILURE
        if self.index == 0:
            status = self.children[0].tick()
            if status == RUNNING:
                return RUNNING
            self.children[0].reset()
            if status == SUCCESS:
                return SUCCESS
            if len(self.children) == 1:
                return FAILURE
            self.log_warn(f'{self.children[0].name} failed - recovery')
            self.index = 1
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                return FAILURE
            self.index += 1
        self.log_info('recovered - carrying on')
        return SUCCESS

    def get_active_node_ids(self):
        if self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self.index = 0


class WhileHolding(BaseControl):
    """Children in order while the gripper holds something (holding=true) -
    or holds nothing (holding=false). The moment that changes, the children
    are stopped (reset: a Nav2 goal is cancelled) and it fails."""

    def __init__(self, node, holding: bool = True):
        super().__init__(node, name='WhileHolding')
        self.om = omniman(node)
        self.want = bool(holding)
        self.index = 0

    def tick(self):
        if self.om.holding != self.want:
            self.log_warn(f'{"lost the object" if self.want else "holding something"}: '
                          f'{self.om.gripper} - stopping')
            _reset_all(self.children)
            self.index = 0
            return FAILURE
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                return FAILURE
            self.index += 1
        return SUCCESS

    def get_active_node_ids(self):
        if self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self.index = 0


class WithControlLock(BaseControl):
    """Take the control lock as `owner`, run the children in order, give it
    back - when they end, fail, or the tree is stopped. Waits for whoever
    holds the lock (up to wait_s), never takes it by force."""

    def __init__(self, node, owner: str = 'policy', wait_s: float = 30.0):
        super().__init__(node, name='WithControlLock')
        self.om = omniman(node)
        self.owner = str(owner)
        self.wait_s = float(wait_s)
        self._clear()

    def _clear(self):
        self.held, self.future, self.index = False, None, 0
        self.since = None

    def tick(self):
        if not self.held:
            now = time.monotonic()
            self.since = self.since or now
            if self.future is None:
                if not self.om.acquire.service_is_ready():
                    if now - self.since > self.wait_s:
                        self.log_warn('control_arbiter not answering (/control/acquire)')
                        return FAILURE
                    return RUNNING
                req = AcquireControl.Request()
                req.owner = self.owner
                req.node = self.node.get_fully_qualified_name()
                self.future = self.om.acquire.call_async(req)
                return RUNNING
            if not self.future.done():
                return RUNNING
            res, self.future = self.future.result(), None
            if res is None or not res.success:
                if now - self.since > self.wait_s:
                    self.log_warn(f'no control as "{self.owner}" within {self.wait_s:.0f}s: '
                                  f'{res.message if res else "no answer"}')
                    return FAILURE
                return RUNNING
            self.held = True
            self.log_info(f'control taken as "{self.owner}"')
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                self._give_back()
                return FAILURE
            self.index += 1
        self._give_back()
        return SUCCESS

    def _give_back(self):
        om, owner, future = self.om, self.owner, self.future
        if future is not None and not future.done():      # a late "yes"
            future.add_done_callback(
                lambda f: f.result() is not None and f.result().success and om.release(owner))
        if self.held:
            om.release(owner)
            self.log_info(f'control "{owner}" given back')
        self.held, self.future = False, None

    def get_active_node_ids(self):
        if self.held and self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return [self.uid]

    def reset(self):
        super().reset()       # children first: they stop what they drive
        self._give_back()
        self._clear()
