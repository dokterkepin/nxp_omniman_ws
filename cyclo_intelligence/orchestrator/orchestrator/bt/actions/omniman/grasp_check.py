#!/usr/bin/env python3
"""OmnimanGraspCheck: grasp_monitor, is the gripper holding (or not)."""

import time

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import RUNNING, _Step


class OmnimanGraspCheck(_Step, BaseAction):
    """grasp_monitor: SUCCESS when the gripper holds something
    (holding=true: a pick worked) or, with holding=false, when it does not
    (a place let go). within_s gives a gripper still moving that long to get
    there; 0 = read it once."""

    def __init__(self, node, holding: bool = True, within_s: float = 0.0):
        super().__init__(node, name='OmnimanGraspCheck')
        self._start(node)
        self.want = bool(holding)
        self.within_s = float(within_s)
        self.since = time.monotonic()
        self.first = True

    def tick(self):
        if self.first:
            self.since, self.first = time.monotonic(), False
        if self.om.holding == self.want:
            return self.succeed(self.om.gripper)
        if time.monotonic() - self.since < self.within_s:
            return RUNNING
        return self.fail(f'{"not holding" if self.want else "still holding"}: '
                         f'{self.om.gripper}')

    def reset(self):
        super().reset()
        self.first = True
