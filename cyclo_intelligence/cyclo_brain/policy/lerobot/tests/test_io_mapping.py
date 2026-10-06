#!/usr/bin/env python3

import sys
import types
import unittest
import importlib.util
from pathlib import Path


robot_client_stub = types.ModuleType("robot_client")
robot_client_stub.RobotClient = object
sys.modules.setdefault("robot_client", robot_client_stub)

ENGINE_DIR = Path(__file__).resolve().parents[1] / "lerobot_engine"
package = types.ModuleType("lerobot_engine")
package.__path__ = [str(ENGINE_DIR)]
sys.modules.setdefault("lerobot_engine", package)

spec = importlib.util.spec_from_file_location(
    "lerobot_engine.io_mapping",
    ENGINE_DIR / "io_mapping.py",
)
io_mapping = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = io_mapping
spec.loader.exec_module(io_mapping)
IoMappingMixin = io_mapping.IoMappingMixin


class IoMappingCameraAliasTest(unittest.TestCase):
    def test_maps_rgb_prefixed_cameras_to_policy_keys(self):
        robot_cameras = [
            "rgb.cam_left_head",
            "rgb.cam_right_head",
            "rgb.cam_left_wrist",
            "rgb.cam_right_wrist",
        ]
        policy_keys = {
            "observation.images.cam_left_head",
            "observation.images.cam_right_head",
            "observation.images.cam_left_wrist",
            "observation.images.cam_right_wrist",
        }

        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(robot_cameras, policy_keys),
            {
                "rgb.cam_left_head": "observation.images.cam_left_head",
                "rgb.cam_right_head": "observation.images.cam_right_head",
                "rgb.cam_left_wrist": "observation.images.cam_left_wrist",
                "rgb.cam_right_wrist": "observation.images.cam_right_wrist",
            },
        )

    def test_keeps_exact_camera_key_preferred(self):
        robot_cameras = ["rgb.cam_left_head"]
        policy_keys = {
            "observation.images.rgb.cam_left_head",
            "observation.images.cam_left_head",
        }

        with self.assertRaisesRegex(RuntimeError, "Missing camera mappings"):
            IoMappingMixin._resolve_camera_mappings(robot_cameras, policy_keys)

        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(
                robot_cameras,
                {"observation.images.rgb.cam_left_head"},
            ),
            {"rgb.cam_left_head": "observation.images.rgb.cam_left_head"},
        )

    def test_maps_legacy_single_head_policy_key_to_left_head_camera(self):
        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(
                [
                    "cam_left_head",
                    "cam_left_wrist",
                    "cam_right_wrist",
                ],
                {
                    "observation.images.rgb.cam_head",
                    "observation.images.cam_wrist_left",
                    "observation.images.cam_wrist_right",
                },
            ),
            {
                "cam_left_head": "observation.images.rgb.cam_head",
                "cam_left_wrist": "observation.images.cam_wrist_left",
                "cam_right_wrist": "observation.images.cam_wrist_right",
            },
        )


if __name__ == "__main__":
    unittest.main()
