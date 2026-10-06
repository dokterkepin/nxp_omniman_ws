#!/usr/bin/env python3
"""OmnimanNavigate: Nav2 to a place in omniman_vla/config/poses.yaml."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from omniman_interfaces.srv import AcquireControl
from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import (
    CONFIG, FAILURE, RUNNING, SERVICE_WAIT_S, SUCCESS, _read_yaml, _Step, _text
)


class OmnimanNavigate(_Step, BaseAction):
    """Drive with Nav2 to a place saved in omniman_vla/config/poses.yaml
    (the web UI's "Save here"). Takes the control lock as "nav" while
    driving, waits for the base to be still, gives the lock back."""

    def __init__(self, node, place: str = 'pick_area', settle_timeout_s: float = 5.0,
                 timeout_s: float = 0.0):
        super().__init__(node, name='OmnimanNavigate')
        self._start(node)
        self.place = _text(place)
        self.settle_timeout_s = float(settle_timeout_s)
        self.timeout_s = float(timeout_s)         # 0 = until Nav2 says
        self._clear()

    def _clear(self):
        self.phase, self.future, self.goal, self.result = 'acquire', None, None, None
        self.since = time.monotonic()
        self.noted = 0.0

    def tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'acquire':
            if self.future is None:
                req = AcquireControl.Request()
                req.owner = 'nav'
                req.node = self.node.get_fully_qualified_name()
                self.future = self.call(om.acquire, req)
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('control_arbiter not answering (/control/acquire)')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res, self.future = self.future.result(), None
            if res is None or not res.success:
                if now - self.noted > 5.0:      # someone else drives: wait, ask again
                    self.log_info(f'waiting for control: {res.message if res else "no answer"}')
                    self.noted = now
                return RUNNING
            self.phase, self.since = 'owner', now
            return RUNNING

        if self.phase == 'owner':
            # /control/owner must say so too, or the takeover check below
            # would read an older "nobody".
            if om.owner == 'nav' or now - self.since > 3.0:
                self.phase, self.since = 'goal', now
            return RUNNING

        if self.phase == 'goal':
            if not om.nav.server_is_ready():
                if now - self.since > SERVICE_WAIT_S:
                    self._end()
                    return self.fail('Nav2 not answering (navigate_to_pose)')
                return RUNNING
            try:
                pose = _read_yaml('poses.yaml')[self.place]
            except (OSError, KeyError) as e:
                self._end()
                return self.fail(f'no place "{self.place}" in {CONFIG / "poses.yaml"} ({e})')
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(pose['x'])
            goal.pose.pose.position.y = float(pose['y'])
            yaw = math.radians(float(pose['yaw']))
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.log_info(f'nav to {self.place} (x={pose["x"]:.2f}, y={pose["y"]:.2f}, '
                          f'yaw={pose["yaw"]:.0f})')
            self.future = om.nav.send_goal_async(goal)
            self.phase, self.since = 'accepted', now
            return RUNNING

        if self.phase == 'accepted':
            if not self.future.done():
                return RUNNING
            self.goal = self.future.result()
            if self.goal is None or not self.goal.accepted:
                self._end()
                return self.fail('Nav2 refused the goal')
            self.result = self.goal.get_result_async()
            self.phase = 'drive'
            return RUNNING

        if self.phase == 'drive':
            if om.owner != 'nav':
                self._end()
                return self.fail(f'control taken by {om.owner or "a forced release"}')
            if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
                self._end()
                return self.fail(f'Nav2 did not arrive within {self.timeout_s:.0f}s')
            if not self.result.done():
                return RUNNING
            status = self.result.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._end()
                return self.fail(f'Nav2 did not reach {self.place} (goal status {status})')
            self.phase, self.since = 'settle', now
            return RUNNING

        if self.phase == 'settle':
            status = self.settle(self.since, self.settle_timeout_s)
            if status == RUNNING:
                return RUNNING
            self._end()
            return self.succeed(f'arrived at {self.place}') if status == SUCCESS else status
        return FAILURE

    def _end(self):
        """Cancel a goal still running, give the lock back - also for an
        answer still on its way, so a late "acquired" or "goal accepted"
        cannot leave the lock held or the base driving."""
        om, future = self.om, self.future
        if self.phase == 'acquire' and future not in (None, 'gone') and not future.done():
            future.add_done_callback(
                lambda f: f.result() is not None and f.result().success and om.release('nav'))
        if self.phase == 'accepted' and not future.done():
            def cancel_late(f):
                goal = f.result()
                if goal is not None and goal.accepted:
                    goal.cancel_goal_async()
                om.release('nav')
            future.add_done_callback(cancel_late)
        elif self.phase == 'drive' and self.result is not None and not self.result.done():
            self.goal.cancel_goal_async()
        if self.phase in ('owner', 'goal', 'drive', 'settle'):
            om.release('nav')
        self.phase = 'done'

    def reset(self):
        super().reset()
        if self.phase not in ('acquire', 'done'):
            self.log_warn('stopped - navigation cancelled')
        self._end()
        self._clear()
