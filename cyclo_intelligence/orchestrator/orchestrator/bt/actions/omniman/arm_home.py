#!/usr/bin/env python3
"""OmnimanArmHome: waits until the arm is home for dwell_s.

The end of a run started with Cyclo's SendCommand.
"""

import time

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import (
    ARM_JOINTS, CONFIG, RUNNING, SERVICE_WAIT_S, _read_yaml, _Step
)


class OmnimanArmHome(_Step, BaseAction):
    """Wait until the arm is at home - every arm joint within `tolerance`
    rad of home_pose in policy_runner.yaml - for dwell_s in a row. Use it
    after Cyclo's SendCommand RESUME to know a GR00T / LeRobot 0.6 run is
    done (then SendCommand STOP). Fails after timeout_s."""

    def __init__(self, node, dwell_s: float = 7.0, tolerance: float = 0.2,
                 timeout_s: float = 90.0):
        super().__init__(node, name='OmnimanArmHome')
        self._start(node)
        self.dwell_s = float(dwell_s)
        self.tolerance = float(tolerance)
        self.timeout_s = float(timeout_s)         # 0 = wait for ever
        params = (_read_yaml('policy_runner.yaml').get('policy_runner') or {}) \
            .get('ros__parameters') or {}
        self.home = [float(v) for v in params.get('home_pose', [])]
        if len(self.home) != len(ARM_JOINTS):
            raise ValueError(f'OmnimanArmHome: home_pose in {CONFIG / "policy_runner.yaml"} '
                             f'must have {len(ARM_JOINTS)} values')
        self._clear()

    def _clear(self):
        self.since = time.monotonic()
        self.home_since = None
        self.noted = 0.0

    def tick(self):
        now, joints = time.monotonic(), self.om.joints
        if any(j not in joints for j in ARM_JOINTS):
            if now - self.since > SERVICE_WAIT_S:
                return self.fail('no arm joints on /joint_states')
            return RUNNING
        worst = max(abs(joints[j] - h) for j, h in zip(ARM_JOINTS, self.home))
        if worst > self.tolerance:
            self.home_since = None
        elif self.home_since is None:
            self.home_since = now
        if self.home_since is not None and now - self.home_since >= self.dwell_s:
            return self.succeed(f'arm home for {self.dwell_s:g}s (worst joint {worst:.3f} rad)')
        if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
            return self.fail(f'arm not home for {self.dwell_s:g}s within '
                             f'{self.timeout_s:.0f}s (worst joint {worst:.3f} rad)')
        return RUNNING

    def reset(self):
        super().reset()
        self._clear()
