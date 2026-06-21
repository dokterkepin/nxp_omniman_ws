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

"""Loader for behavior trees from XML files."""

import xml.etree.ElementTree as ET  # noqa: I100
from typing import Dict  # noqa: I100
from typing import TYPE_CHECKING  # noqa: I100
from typing import Type  # noqa: I100

from physical_ai_bt.actions import MoveArms
from physical_ai_bt.actions import MoveHead
from physical_ai_bt.actions import MoveLift
from physical_ai_bt.actions import Rotate
from physical_ai_bt.actions.base_action import BaseAction
from physical_ai_bt.bt_core import BTNode
from physical_ai_bt.controls import Sequence
from physical_ai_bt.controls.base_control import BaseControl

if TYPE_CHECKING:
    from rclpy.node import Node


class TreeLoader:
    """Loads behavior trees from XML files and instantiates nodes."""

    def __init__(
        self, node: 'Node', joint_names: list = None, topic_config: dict = None
    ):
        """Initialize the tree loader."""
        self.node = node
        self.joint_names = joint_names or []
        self.topic_config = topic_config or {}

        self.control_types: Dict[str, Type[BaseControl]] = {
            'Sequence': Sequence,
        }

        self.action_types: Dict[str, Type[BaseAction]] = {
            'Rotate': Rotate,
            'MoveHead': MoveHead,
            'MoveArms': MoveArms,
            'MoveLift': MoveLift,
        }

    def load_tree_from_file(
        self, xml_path: str, main_tree_id: str = None
    ) -> BTNode:
        """Load a behavior tree from an XML file."""
        tree = ET.parse(xml_path)
        root = tree.getroot()

        if main_tree_id is None:
            main_tree_id = root.get('main_tree_to_execute')
            if not main_tree_id:
                raise ValueError(
                    'No main_tree_to_execute specified in XML'
                )

        for behavior_tree in root.findall('BehaviorTree'):
            if behavior_tree.get('ID') == main_tree_id:
                return self._load_node(behavior_tree[0])

        raise ValueError(
            f"BehaviorTree with ID '{main_tree_id}' not found"
        )

    def _load_node(self, xml_node: ET.Element) -> BTNode:
        """Load a behavior tree node from an XML element."""
        node_type = xml_node.tag
        node_id = xml_node.get('ID', node_type)
        node_name = xml_node.get('name', node_id)

        if node_type in self.control_types:
            control_class = self.control_types[node_type]
            control_node = control_class(self.node, name=node_name)

            for child_xml in xml_node:
                child_node = self._load_node(child_xml)
                control_node.add_child(child_node)

            return control_node

        elif node_id in self.action_types:
            action_class = self.action_types[node_id]
            params = self._parse_node_params(xml_node)
            return self._create_action(action_class, node_name, params)

        else:
            raise ValueError(
                f"Unknown node type '{node_type}' with ID '{node_id}'"
            )

    def _parse_node_params(self, xml_node: ET.Element) -> Dict:
        """Parse parameters from XML node attributes."""
        params = {}

        for key, value in xml_node.attrib.items():
            if key not in ['ID', 'name']:
                params[key] = self._convert_value(value)

        return params

    def _convert_value(self, value: str):
        """Convert string value to appropriate Python type."""
        if value.lower() in ('true', 'false'):
            return value.lower() == 'true'

        try:
            if '.' in value:
                return float(value)
            return int(value)
        except ValueError:
            pass

        if ',' in value:
            parts = [p.strip() for p in value.split(',')]
            try:
                return [float(p) if '.' in p else int(p) for p in parts]
            except ValueError:
                return parts

        return value

    def _get_joint_names_for_group(self, group_name: str) -> list:
        """Get joint names for a specific joint group from topic_config."""
        if not self.topic_config or 'joint_order' not in self.topic_config:
            return []

        joint_order = self.topic_config['joint_order']
        return joint_order.get(group_name, [])

    def _create_action(
        self, action_class: Type[BaseAction], name: str, params: Dict
    ) -> BaseAction:
        """Create an action node instance with the given parameters."""
        if action_class == Rotate:
            return action_class(
                node=self.node,
                angle_deg=params.get('angle_deg', 90.0),
                topic_config=self.topic_config
            )

        elif action_class == MoveHead:
            head_joints = self._get_joint_names_for_group('leader_head')

            return action_class(
                node=self.node,
                head_positions=params.get('head_positions', [0.0, 0.0]),
                head_joint_names=head_joints if head_joints else None,
                position_threshold=params.get('position_threshold', 0.01),
                duration=params.get('duration', 5.0)
            )

        elif action_class == MoveArms:
            default_positions = [0.0] * 8
            left_joints = self._get_joint_names_for_group('leader_left')
            right_joints = self._get_joint_names_for_group('leader_right')

            return action_class(
                node=self.node,
                left_positions=params.get('left_positions', default_positions),
                right_positions=params.get(
                    'right_positions', default_positions
                ),
                left_joint_names=left_joints if left_joints else None,
                right_joint_names=right_joints if right_joints else None,
                position_threshold=params.get('position_threshold', 0.01),
                duration=params.get('duration', 2.0)
            )

        elif action_class == MoveLift:
            lift_joints = self._get_joint_names_for_group('leader_lift')
            lift_joint_name = lift_joints[0] if lift_joints else None

            return action_class(
                node=self.node,
                lift_position=params.get('lift_position', 0.0),
                lift_joint_name=lift_joint_name,
                position_threshold=params.get('position_threshold', 0.01),
                duration=params.get('duration', 5.0)
            )

        else:
            raise ValueError(f'Unknown action class: {action_class}')
