#!/usr/bin/env python3
"""What the omniman controls share."""

from orchestrator.bt.bt_core import NodeStatus

RUNNING, SUCCESS, FAILURE = NodeStatus.RUNNING, NodeStatus.SUCCESS, NodeStatus.FAILURE


def _reset_all(children):
    for child in children:
        child.reset()
