#!/usr/bin/env python3
"""OmnimanPolicy: an arm policy through policy_runner, until the arm is home."""

import os
import time

from omniman_interfaces.srv import RunPolicy
from orchestrator.bt.actions.base_action import BaseAction
from std_srvs.srv import Trigger
from orchestrator.bt.actions.omniman.common import (
    FAILURE, RUNNING, SUCCESS, _Step, _text
)
from orchestrator.bt.actions.omniman.policy_info import policy_info


class OmnimanPolicy(_Step, BaseAction):
    """Run an arm policy through omniman's policy_runner until the arm is back
    home. `policy_path` empty = policy_runner.yaml's default. timeout_s
    stops a policy that never settles at home, and also gives up waiting to
    start (see below).

    It starts only when the base is still, the arm has finished its last move
    and a camera frame has arrived after both. It logs the checkpoint first:
    path, policy type, weights, device and the chunk_size / n_action_steps /
    temporal_ensemble_coeff of its config.json. Its end is a result seen after
    the run was busy - there is no timed fallback."""

    def __init__(self, node, instruction: str = 'pick the object', policy_path: str = '',
                 timeout_s: float = 90.0):
        super().__init__(node, name='OmnimanPolicy')
        self._start(node)
        self.instruction = _text(instruction)
        self.policy_path = os.path.expanduser(_text(policy_path)) if policy_path else ''
        self.timeout_s = float(timeout_s)         # 0 = until policy_runner says
        self._clear()

    def _clear(self):
        self.phase, self.future, self.busy_seen = 'fresh', None, False
        self.since = time.monotonic()
        self.announced = False

    def tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'fresh':
            if not self.announced:
                self.log_info(f'policy: "{self.instruction}"')
                for line in policy_info(self.policy_path):
                    self.log_info(line)
                self.announced = True
            status = self.wait_settled(self.since, self.timeout_s)
            if status != SUCCESS:
                return status
            self.phase, self.since = 'call', now
        if self.phase == 'call':
            if self.future is None:
                req = RunPolicy.Request()
                req.policy_path = self.policy_path
                req.instruction = self.instruction
                req.force = False
                self.future = self.call(om.policy_run, req)
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('policy_runner not answering (/policy_runner/run)')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res = self.future.result()
            if res is None or not res.success:
                self.phase = 'done'
                return self.fail(f'policy_runner refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'watch', now
            return RUNNING

        if self.phase == 'watch':
            if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
                self._stop()
                return self.fail(f'policy ran longer than {self.timeout_s:.0f}s '
                                 f'({om.arm}) - policy stopped')
            # starting -> working -> idle; idle after busy is the end. The
            # status is latched, so an idle before that is the last run's.
            if om.policy_status != 'idle':
                self.busy_seen = True
                return RUNNING
            if self.busy_seen:
                self.phase = 'done'
                return self.succeed(f'arm back home after {now - self.since:.0f}s')
            return RUNNING
        return FAILURE

    def _stop(self):
        if self.phase == 'watch' and self.om.policy_stop.service_is_ready():
            self.om.policy_stop.call_async(Trigger.Request())
        self.phase = 'done'

    def reset(self):
        super().reset()
        if self.phase == 'watch':
            self.log_warn('stopped - policy stopped')
        self._stop()
        self._clear()
