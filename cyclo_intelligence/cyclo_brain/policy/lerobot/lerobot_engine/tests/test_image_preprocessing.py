#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "image_preprocessing.py"
spec = importlib.util.spec_from_file_location("image_preprocessing", MODULE_PATH)
image_preprocessing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(image_preprocessing)


class ImagePreprocessingTest(unittest.TestCase):
    def test_extracts_per_policy_image_resize_targets(self):
        features = {
            "observation.state": SimpleNamespace(shape=(22,)),
            "observation.images.cam_head_left": SimpleNamespace(shape=(3, 720, 1280)),
            "observation.images.cam_wrist_left": SimpleNamespace(shape=(3, 640, 480)),
        }

        targets = image_preprocessing.infer_image_resize_targets(features)

        self.assertEqual(
            targets,
            {
                "observation.images.cam_head_left": (1280, 720),
                "observation.images.cam_wrist_left": (480, 640),
            },
        )

    def test_rotates_wrist_image_from_640x480_to_480x640(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        rotated = image_preprocessing.apply_rotation(image, 270)

        self.assertEqual(rotated.shape, (640, 480, 3))


if __name__ == "__main__":
    unittest.main()
