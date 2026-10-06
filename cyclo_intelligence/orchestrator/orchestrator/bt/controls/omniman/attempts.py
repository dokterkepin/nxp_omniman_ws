#!/usr/bin/env python3
"""Attempts: children in order; if one fails, all of them again, up to max_attempts tries."""

from orchestrator.bt.controls.base_control import BaseControl
from orchestrator.bt.controls.omniman.common import FAILURE, RUNNING, SUCCESS, _reset_all


class Attempts(BaseControl):
    """Children in order, like Sequence. If one fails, every child is reset
    and they run again from the first - up to max_attempts tries in all;
    then it fails."""

    def __init__(self, node, max_attempts: int = 3):
        super().__init__(node, name='Attempts')
        self.max_attempts = max(1, int(max_attempts))
        self.index = 0
        self.failures = 0

    def tick(self):
        if not self.children:
            return FAILURE
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                self.failures += 1
                if self.failures >= self.max_attempts:
                    self.log_warn(f'{child.name} failed - {self.failures} of '
                                  f'{self.max_attempts} tries used, giving up')
                    return FAILURE
                self.log_info(f'{child.name} failed - try {self.failures + 1} of '
                              f'{self.max_attempts}')
                _reset_all(self.children)
                self.index = 0
                return RUNNING
            self.index += 1
        return SUCCESS

    def get_active_node_ids(self):
        if self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self.index = 0
        self.failures = 0
