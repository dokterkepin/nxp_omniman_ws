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

"""Control nodes for Behavior Tree."""

__all__ = ['Loop', 'Sequence']


def __getattr__(name):
    """Lazily expose built-in controls."""
    if name == 'Loop':
        from orchestrator.bt.controls.loop import Loop
        return Loop
    if name == 'Sequence':
        from orchestrator.bt.controls.sequence import Sequence
        return Sequence
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
