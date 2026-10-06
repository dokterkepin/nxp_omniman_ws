#!/usr/bin/env python3
"""WhileHolding: children in order while grasp_monitor says the gripper is holding."""

from orchestrator.bt.actions.omniman.common import omniman
from orchestrator.bt.controls.base_control import BaseControl
from orchestrator.bt.controls.omniman.common import FAILURE, RUNNING, SUCCESS, _reset_all


class WhileHolding(BaseControl):
    """Children in order while the gripper holds something (holding=true) -
    or holds nothing (holding=false). The moment that changes, the children
    are stopped (reset: a Nav2 goal is cancelled) and it fails."""

    def __init__(self, node, holding: bool = True):
        super().__init__(node, name='WhileHolding')
        self.om = omniman(node)
        self.want = bool(holding)
        self.index = 0

    def tick(self):
        if self.om.holding != self.want:
            self.log_warn(f'{"lost the object" if self.want else "holding something"}: '
                          f'{self.om.gripper} - stopping')
            _reset_all(self.children)
            self.index = 0
            return FAILURE
        while self.index < len(self.children):
            child = self.children[self.index]
            status = child.tick()
            if status == RUNNING:
                return RUNNING
            child.reset()
            if status == FAILURE:
                return FAILURE
            self.index += 1
        return SUCCESS

    def get_active_node_ids(self):
        if self.index < len(self.children):
            return self.children[self.index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self.index = 0
