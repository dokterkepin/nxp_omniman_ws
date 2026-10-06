#!/usr/bin/env python3
"""OnFailure: the first child is the work; if it fails, the other children run as the recovery."""

from orchestrator.bt.controls.base_control import BaseControl
from orchestrator.bt.controls.omniman.common import FAILURE, RUNNING, SUCCESS


class OnFailure(BaseControl):
    """The first child is the work. If it fails, the other children run in
    order: the recovery - any nodes (drive somewhere, run a policy, ...).
    SUCCESS if the work succeeded, or else if the recovery did; FAILURE if
    the recovery failed too - end it with StartAgain to always make an
    enclosing Attempts start over."""

    def __init__(self, node):
        super().__init__(node, name='OnFailure')
        self.index = 0          # 0 = the work; 1.. = the recovery

    def tick(self):
        if not self.children:
            return FAILURE
        if self.index == 0:
            status = self.children[0].tick()
            if status == RUNNING:
                return RUNNING
            self.children[0].reset()
            if status == SUCCESS:
                return SUCCESS
            if len(self.children) == 1:
                return FAILURE
            self.log_warn(f'{self.children[0].name} failed - recovery')
            self.index = 1
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                return FAILURE
            self.index += 1
        self.log_info('recovered - carrying on')
        return SUCCESS

    def get_active_node_ids(self):
        if self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self.index = 0
