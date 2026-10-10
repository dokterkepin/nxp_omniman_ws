#!/usr/bin/env python3
"""OmnimanGraspCheck: grasp_monitor, is the gripper holding (or not)."""

import time

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import RUNNING, _Step


class OmnimanGraspCheck(_Step, BaseAction):
    """grasp_monitor: SUCCESS when the gripper holds something
    (holding=true: a pick worked) or, with holding=false, when it does not
    (a place let go).

    grasp_monitor only changes its answer once a reading has held for stable_s,
    so right after a policy ends it can still be one step behind a gripper that
    is closing or opening. This waits until grasp_monitor agrees with what the
    gripper joint reads right now, then judges. It fails only if they never
    agree within `timeout_s` (0 = holding_timeout_s in mission.yaml)."""

    def __init__(self, node, holding: bool = True, timeout_s: float = 0.0):
        super().__init__(node, name='OmnimanGraspCheck')
        self._start(node)
        self.want = bool(holding)
        self.timeout_s = float(timeout_s)
        self.since = time.monotonic()
        self.first = True

    def tick(self):
        om = self.om
        if self.first:
            self.since, self.first = time.monotonic(), False
        reads = om.gripper_reads_holding()
        if reads is None or reads != om.holding:
            limit = om.setting('holding_timeout_s', self.timeout_s)
            if time.monotonic() - self.since > limit:
                return self.fail(f'grasp_monitor did not agree with the gripper within '
                                 f'{limit:g}s (it says: {om.gripper})')
            return RUNNING
        if om.holding == self.want:
            return self.succeed(om.gripper)
        return self.fail(f'{"not holding" if self.want else "still holding"}: {om.gripper}')

    def reset(self):
        super().reset()
        self.first = True
