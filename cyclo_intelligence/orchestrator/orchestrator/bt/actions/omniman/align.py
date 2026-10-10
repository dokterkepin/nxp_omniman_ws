#!/usr/bin/env python3
"""OmnimanAlign: base correction with visual_align on what the SAM detector finds."""

import time

from orchestrator.bt.actions.base_action import BaseAction
from std_msgs.msg import String
from std_srvs.srv import Trigger
from orchestrator.bt.actions.omniman.common import (
    FAILURE, RUNNING, _Step, _text
)


class OmnimanAlign(_Step, BaseAction):
    """Base correction: visual_align moves the base until the detector's
    `target` sits where the policy expects it (aim point in
    visual_align.yaml), then waits for the base to be still. `target` is any
    text - it is the SAM detector's prompt, e.g. "yellow cup lid". When the node
    ends the prompt is cleared, so the detector goes idle (it only shows the
    camera picture) until the next one."""

    def __init__(self, node, target: str = 'yellow cup lid', timeout_s: float = 60.0,
                 settle_timeout_s: float = 5.0):
        super().__init__(node, name='OmnimanAlign')
        self._start(node)
        self.target = _text(target)
        self.timeout_s = float(timeout_s)         # 0 = until visual_align says
        self.settle_timeout_s = float(settle_timeout_s)
        self._clear()

    def _clear(self):
        self.phase, self.future, self.busy_seen = 'start', None, False
        self.since = time.monotonic()
        self.active = False         # the detector has our prompt until _release_prompt()

    def _release_prompt(self):
        if self.active:
            self.om.target_pub.publish(String(data=''))
            self.active = False

    def tick(self):
        status = self._tick()
        if status != RUNNING:
            self._release_prompt()
        return status

    def _tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'start':
            om.target_pub.publish(String(data=self.target))
            self.active = True
            self.log_info(f'align to "{self.target}"')
            self.phase, self.since = 'call', now
        if self.phase == 'call':
            if self.future is None:
                self.future = self.call(om.align_run, Trigger.Request())
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('visual_align not answering (/visual_align/run)')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res = self.future.result()
            if res is None or not res.success:
                self.phase = 'done'
                return self.fail(f'visual_align refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'align', now
            return RUNNING

        if self.phase == 'align':
            # The status is latched: an old "aligned" is there before this
            # run starts, so only a result after busy counts.
            status = om.align_status
            if status in ('starting', 'searching', 'aligning'):
                self.busy_seen = True
            elif self.busy_seen and status == 'aligned':
                self.log_info(f'aligned in {now - self.since:.1f}s')
                self.phase, self.since = 'settle', now
                return RUNNING
            elif self.busy_seen and status.startswith('failed'):
                self.phase = 'done'
                return self.fail(f'visual_align {status}')
            if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
                self._stop()
                return self.fail(f'no result within {self.timeout_s:.0f}s '
                                 f'(last: {status or "nothing"}) - align stopped')
            return RUNNING

        if self.phase == 'settle':
            status = self.settle(self.since, self.settle_timeout_s)
            if status != RUNNING:
                self.phase = 'done'
            return status
        return FAILURE

    def _stop(self):
        if self.phase == 'align' and self.om.align_stop.service_is_ready():
            self.om.align_stop.call_async(Trigger.Request())
        self.phase = 'done'

    def reset(self):
        super().reset()
        if self.phase == 'align':
            self.log_warn('stopped - align stopped')
        self._stop()
        self._release_prompt()
        self._clear()
