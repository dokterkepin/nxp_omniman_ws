#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Seongwoo Kim

"""Template for a custom BT control node.

Copy this file to:

    orchestrator/orchestrator/bt/controls/my_control.py

Then rename `MyControl`, adjust constructor parameters, implement tick(), and
press "Refresh Nodes" in BT Manager.
"""

from typing import TYPE_CHECKING

from orchestrator.bt.bt_core import NodeStatus
from orchestrator.bt.controls.base_control import BaseControl

if TYPE_CHECKING:
    from rclpy.node import Node


class MyControl(BaseControl):
    def __init__(
        self,
        node: 'Node',
        name: str = 'MyControl',
        fail_fast: bool = True,
    ):
        super().__init__(node, name)
        self.fail_fast = bool(fail_fast)
        self._current_child_index = 0

    def tick(self) -> NodeStatus:
        if not self.children:
            self.log_error('No child node')
            return NodeStatus.FAILURE

        while self._current_child_index < len(self.children):
            child = self.children[self._current_child_index]
            status = child.tick()

            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            if status == NodeStatus.FAILURE and self.fail_fast:
                child.reset()
                return NodeStatus.FAILURE

            child.reset()
            self._current_child_index += 1

        return NodeStatus.SUCCESS

    def get_active_node_ids(self):
        if self._current_child_index < len(self.children):
            return self.children[self._current_child_index].get_active_node_ids()
        return []

    def reset(self):
        super().reset()
        self._current_child_index = 0
