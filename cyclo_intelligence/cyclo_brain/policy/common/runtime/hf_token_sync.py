#!/usr/bin/env python3
"""Sync Cyclo's Hugging Face endpoint token into the standard HF cache."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional


HF_ENDPOINT = "https://huggingface.co"


def hf_home() -> Path:
    raw = os.environ.get("HF_HOME")
    return Path(raw) if raw else Path.home() / ".cache" / "huggingface"


def endpoint_store_path() -> Path:
    raw = os.environ.get("CYCLO_HF_ENDPOINT_STORE")
    return Path(raw) if raw else hf_home() / "hf_endpoints.json"


def _env_token() -> str:
    for name in ("HF_TOKEN", "HUGGINGFACE_HUB_TOKEN", "HUGGING_FACE_HUB_TOKEN"):
        token = os.environ.get(name, "").strip()
        if token:
            return token
    return ""


def _token_from_endpoint_store(path: Path) -> str:
    if not path.exists():
        return ""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return ""

    endpoints = data.get("endpoints") or {}
    preferred = endpoints.get(HF_ENDPOINT) or endpoints.get(HF_ENDPOINT + "/")
    if preferred and preferred.get("token"):
        return str(preferred["token"]).strip()

    active = data.get("active") or ""
    active_entry = endpoints.get(active) or endpoints.get(active.rstrip("/"))
    if active_entry and active_entry.get("token"):
        return str(active_entry["token"]).strip()
    return ""


def resolve_token() -> str:
    return _env_token() or _token_from_endpoint_store(endpoint_store_path())


def sync_token_file(token: Optional[str] = None) -> bool:
    token = (token or resolve_token()).strip()
    if not token:
        return False

    token_path = hf_home() / "token"
    token_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        existing = token_path.read_text(encoding="utf-8").strip()
    except OSError:
        existing = ""
    if existing != token:
        token_path.write_text(token + "\n", encoding="utf-8")
    try:
        os.chmod(token_path, 0o600)
    except OSError:
        pass
    return True


def main() -> int:
    if sync_token_file():
        print("HF token available for gated model downloads")
    else:
        print("HF token not found; gated model downloads may fail")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
