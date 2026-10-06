#!/usr/bin/env python3
"""WithControlLock: children in order holding the control lock as `owner`."""

import time

from omniman_interfaces.srv import AcquireControl
from orchestrator.bt.actions.omniman.common import omniman
from orchestrator.bt.controls.base_control import BaseControl
from orchestrator.bt.controls.omniman.common import FAILURE, RUNNING, SUCCESS


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
