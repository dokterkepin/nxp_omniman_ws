#!/usr/bin/env python3
"""StartAgain: always fails.

Last in an OnFailure recovery, it makes an enclosing Attempts run everything again.
"""

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import (
    FAILURE, _Step, _text
)


class StartAgain(_Step, BaseAction):
    """Always fails. Last in an OnFailure recovery, it makes the Attempts
    around the whole mission start it again from the beginning."""

    def __init__(self, node, reason: str = 'start again'):
        super().__init__(node, name='StartAgain')
        self.reason = _text(reason)

    def tick(self):
        self.log_info(self.reason)
        return FAILURE
