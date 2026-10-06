"""Source-tree path fallbacks for runtime modules."""

from pathlib import Path
from typing import Union


def dev_sdk_path(caller_file: Union[str, Path], depth: int, *parts: str) -> str:
    """Return ``<caller's Nth parent>/<parts>`` or an empty string."""
    parents = Path(caller_file).resolve().parents
    if len(parents) <= depth:
        return ""
    return str(parents[depth].joinpath(*parts))
