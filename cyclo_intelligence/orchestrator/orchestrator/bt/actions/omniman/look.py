#!/usr/bin/env python3
"""OmnimanLook: look for a target from where the arm is, without turning the base."""

import time

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import RUNNING, SERVICE_WAIT_S, _Step, _text
from std_msgs.msg import String


class OmnimanLook(_Step, BaseAction):
    """SUCCESS as soon as the SAM detector reports `target`, FAILURE once it has
    reported on `frames` frames without it (0 = look_frames in mission.yaml: a
    count of detector frames, not seconds). The prompt is cleared when the node
    ends, so the detector goes idle. Use it to see whether an arm pose can see
    the target before spending a turn of the base on it."""

    def __init__(self, node, target: str = 'yellow cup lid', frames: int = 0):
        super().__init__(node, name='OmnimanLook')
        self._start(node)
        self.target = _text(target)
        self.frames = int(frames)
        self._clear()

    def _clear(self):
        self.phase, self.active = 'start', False
        self.seen0 = self.hits0 = 0
        self.since = time.monotonic()

    def _release_prompt(self):
        if self.active:
            self.om.target_pub.publish(String(data=''))
            self.active = False

    def tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'start':
            om.target_pub.publish(String(data=self.target))
            self.active = True
            self.seen0, self.hits0, self.since = om.detections_seen, om.detection_hits, now
            self.phase = 'watch'
        frames = om.detections_seen - self.seen0
        limit = int(om.setting('look_frames', self.frames))
        status = RUNNING
        if om.detection_hits > self.hits0 and om.last_hit[0] == self.target:
            status = self.succeed(f'look for "{self.target}": seen from here '
                                  f'(score {om.last_hit[1]:.2f}, {frames} detector frames)')
        elif frames >= limit:
            status = self.fail(f'look for "{self.target}": not seen from here '
                               f'({frames} detector frames)')
        elif frames == 0 and now - self.since > SERVICE_WAIT_S:
            status = self.fail('no detections from /sam_detector/detections - is it running?')
        if status != RUNNING:
            self.phase = 'done'
            self._release_prompt()
        return status

    def reset(self):
        super().reset()
        self._release_prompt()
        self._clear()
