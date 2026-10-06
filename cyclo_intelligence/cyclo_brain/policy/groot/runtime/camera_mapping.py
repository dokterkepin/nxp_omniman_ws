#!/usr/bin/env python3
"""Compatibility wrapper for the shared robot_client camera mapping helper."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path


def _load_shared_camera_mapping():
    candidates = []
    sdk_path = os.environ.get("ROBOT_CLIENT_SDK_PATH")
    if sdk_path:
        candidates.append(Path(sdk_path) / "robot_client" / "camera_mapping.py")

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(
            parent / "sdk" / "robot_client" / "robot_client" / "camera_mapping.py"
        )

    for candidate in candidates:
        if candidate.exists():
            spec = importlib.util.spec_from_file_location(
                "_shared_camera_mapping",
                candidate,
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

    raise ImportError(f"robot_client camera_mapping.py not found in {candidates}")


_shared = _load_shared_camera_mapping()

camera_key_aliases = _shared.camera_key_aliases
resolve_camera_feature_sources = _shared.resolve_camera_feature_sources
resolve_camera_mappings = _shared.resolve_camera_mappings

__all__ = [
    "camera_key_aliases",
    "resolve_camera_feature_sources",
    "resolve_camera_mappings",
]
