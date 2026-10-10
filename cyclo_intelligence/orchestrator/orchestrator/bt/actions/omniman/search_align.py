#!/usr/bin/env python3
"""OmnimanSearchAlign: find a target from the arm's ready poses, then align to it."""

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.align import OmnimanAlign
from orchestrator.bt.actions.omniman.arm_pose import OmnimanArmPose
from orchestrator.bt.actions.omniman.common import FAILURE, RUNNING, _Step, _text
from orchestrator.bt.actions.omniman.look import OmnimanLook


class OmnimanSearchAlign(_Step, BaseAction):
    """Find `target` from the arm poses, then align to it.

    First WITHOUT turning: for each pose in `poses`, move the arm, look, and - if
    the target is seen - align (which then finds it at once). A few seconds per
    pose. Only if no pose sees it from where the base stands, turn: for each
    pose, move the arm and align, which turns the base looking for the target
    (search_turns in visual_align.yaml). Then the arm goes back to the first
    pose, where the policy starts. SUCCESS when a pose found and aligned to it;
    FAILURE if none did. `timeout_s` is each align's.

    Use it twice in a row: the first may find the target from ready2 or ready3,
    whose view is not the policy's; the second aligns again from ready1."""

    def __init__(self, node, target: str = 'yellow cup lid',
                 poses: str = 'ready1, ready2, ready3', timeout_s: float = 60.0,
                 settle_timeout_s: float = 5.0):
        super().__init__(node, name='OmnimanSearchAlign')
        self._start(node)
        self.target = _text(target)
        names = [p.strip() for p in _text(poses).split(',') if p.strip()]
        if not names:
            raise ValueError('OmnimanSearchAlign: poses is empty')

        def align():
            return OmnimanAlign(node, target=self.target, timeout_s=timeout_s,
                                settle_timeout_s=settle_timeout_s)

        # Each plan is a list of nodes run in order; the first plan whose nodes
        # all succeed ends the search.
        self.plans = [[OmnimanArmPose(node, pose=n), OmnimanLook(node, target=self.target),
                       align()] for n in names]
        self.plans += [[OmnimanArmPose(node, pose=n), align()] for n in names]
        self.first_pose = names[0]
        self.final = OmnimanArmPose(node, pose=self.first_pose)
        self._clear()

    def _clear(self):
        self.plan, self.step, self.back = 0, 0, False

    def _reset_plan(self, plan):
        for child in plan:
            child.reset()

    def tick(self):
        if self.back:
            return self.final.tick()
        current = self.plans[self.plan]
        status = current[self.step].tick()
        if status == RUNNING:
            return RUNNING
        current[self.step].reset()
        if status == FAILURE:
            self._reset_plan(current)
            self.plan, self.step = self.plan + 1, 0
            if self.plan >= len(self.plans):
                return self.fail(f'"{self.target}" not found from any arm pose')
            return RUNNING
        self.step += 1
        if self.step >= len(current):
            self.log_info(f'"{self.target}" found and aligned - arm back to {self.first_pose}')
            self.back = True
        return RUNNING

    def reset(self):
        super().reset()
        for plan in self.plans:
            self._reset_plan(plan)
        self.final.reset()
        self._clear()
