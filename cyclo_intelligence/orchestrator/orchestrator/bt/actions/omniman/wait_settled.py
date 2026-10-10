#!/usr/bin/env python3
"""OmnimanWaitSettled: wait until the robot has settled and the camera has caught up."""

import time

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import _Step


class OmnimanWaitSettled(_Step, BaseAction):
    """SUCCESS once the base is still, the arm has finished its last move and a
    camera frame has arrived after both - the picture a policy starts from must
    not be from before the robot settled. Put it before a policy that does not
    check this itself (OmnimanPolicy does), e.g. Cyclo's SendCommand RESUME.
    Waits for ever if `timeout_s` is 0."""

    def __init__(self, node, timeout_s: float = 30.0):
        super().__init__(node, name='OmnimanWaitSettled')
        self._start(node)
        self.timeout_s = float(timeout_s)
        self.since = time.monotonic()
        self.first = True

    def tick(self):
        if self.first:
            self.since, self.first = time.monotonic(), False
        return self.wait_settled(self.since, self.timeout_s)

    def reset(self):
        super().reset()
        self.first = True
