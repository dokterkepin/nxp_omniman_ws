#!/usr/bin/env python3

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

import hf_token_sync  # noqa: E402


class HFTokenSyncTests(unittest.TestCase):
    def test_syncs_active_endpoint_token_to_standard_cache(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            hf_home = Path(tmp)
            store = {
                "active": "https://huggingface.co",
                "endpoints": {
                    "https://huggingface.co": {
                        "label": "Hugging Face",
                        "token": "hf_test_token",
                        "user_id": "tester",
                    }
                },
            }
            (hf_home / "hf_endpoints.json").write_text(
                json.dumps(store), encoding="utf-8"
            )

            with patch.dict(
                "os.environ",
                {
                    "HF_HOME": str(hf_home),
                    "HF_TOKEN": "",
                    "HUGGINGFACE_HUB_TOKEN": "",
                    "HUGGING_FACE_HUB_TOKEN": "",
                },
                clear=False,
            ):
                self.assertTrue(hf_token_sync.sync_token_file())

            self.assertEqual(
                (hf_home / "token").read_text(encoding="utf-8").strip(),
                "hf_test_token",
            )

    def test_environment_token_takes_precedence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            hf_home = Path(tmp)
            (hf_home / "hf_endpoints.json").write_text(
                json.dumps(
                    {
                        "active": "https://huggingface.co",
                        "endpoints": {
                            "https://huggingface.co": {
                                "token": "hf_store_token",
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )

            with patch.dict(
                "os.environ",
                {
                    "HF_HOME": str(hf_home),
                    "HF_TOKEN": "hf_env_token",
                },
                clear=False,
            ):
                self.assertTrue(hf_token_sync.sync_token_file())

            self.assertEqual(
                (hf_home / "token").read_text(encoding="utf-8").strip(),
                "hf_env_token",
            )


if __name__ == "__main__":
    unittest.main()
