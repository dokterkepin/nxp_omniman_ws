#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


GROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GROOT_DIR))

from runtime.camera_mapping import resolve_camera_mappings


class GrootCameraMappingTest(unittest.TestCase):
    def test_maps_rgb_prefixed_camera_to_policy_key(self):
        self.assertEqual(
            resolve_camera_mappings(
                ["rgb.cam_left_head", "rgb.cam_right_wrist"],
                ["cam_left_head", "cam_right_wrist"],
            ),
            {
                "rgb.cam_left_head": "cam_left_head",
                "rgb.cam_right_wrist": "cam_right_wrist",
            },
        )

    def test_keeps_canonical_camera_names_exact(self):
        self.assertEqual(
            resolve_camera_mappings(
                ["cam_left_head", "cam_left_wrist"],
                ["cam_left_head", "cam_left_wrist"],
            ),
            {
                "cam_left_head": "cam_left_head",
                "cam_left_wrist": "cam_left_wrist",
            },
        )

    def test_maps_full_dataset_key_to_canonical_robot_source(self):
        self.assertEqual(
            resolve_camera_mappings(
                ["cam_left_head"],
                ["observation.images.rgb.cam_left_head"],
            ),
            {
                "cam_left_head": "observation.images.rgb.cam_left_head",
            },
        )

    def test_rejects_two_policy_keys_for_one_robot_source(self):
        with self.assertRaisesRegex(RuntimeError, "matched multiple model keys"):
            resolve_camera_mappings(
                ["rgb.cam_left_head"],
                ["rgb.cam_left_head", "cam_left_head"],
            )


if __name__ == "__main__":
    unittest.main()
