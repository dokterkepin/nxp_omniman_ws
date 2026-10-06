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

"""Stable base class for behavior tree control nodes.

User-defined controls should subclass :class:`BaseControl`, implement
``tick()``, and expose user-editable XML parameters as constructor kwargs.
Copy a template from ``orchestrator.bt.templates`` when creating a new
control; this file is the inheritance API, not the user-facing template.
"""

from typing import TYPE_CHECKING

from orchestrator.bt.bt_core import BTNode

if TYPE_CHECKING:
    from rclpy.node import Node


class BaseControl(BTNode):
    """Base class for nodes that own and tick child BT nodes."""

    def __init__(self, node: 'Node', name: str):
        """Initialize a control node."""
        super().__init__(node, name)
        self.children: list[BTNode] = []

    def add_child(self, child: BTNode):
        """Add a child node to this control node."""
        self.children.append(child)

    def reset(self):
        """Reset the control node and all its children."""
        super().reset()
        for child in self.children:
            child.reset()
