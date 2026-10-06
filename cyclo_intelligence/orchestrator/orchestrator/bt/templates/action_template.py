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

"""Template for a custom BT action node.

Copy this file to:

    orchestrator/orchestrator/bt/actions/my_action.py

Then rename `MyAction`, adjust constructor parameters, implement tick(), and
press "Refresh Nodes" in BT Manager.
"""

import time
from typing import TYPE_CHECKING

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.bt_core import NodeStatus

if TYPE_CHECKING:
    from rclpy.node import Node


class MyAction(BaseAction):
    def __init__(self, node: 'Node', duration: float = 1.0):
        super().__init__(node, name='MyAction')
        self.duration = float(duration)
        self._start_time = None

    def tick(self) -> NodeStatus:
        if self._start_time is None:
            self._start_time = time.monotonic()
            self.log_info(f'Started for {self.duration}s')
            return NodeStatus.RUNNING

        if time.monotonic() - self._start_time >= self.duration:
            self.log_info('Finished')
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        self._start_time = None
