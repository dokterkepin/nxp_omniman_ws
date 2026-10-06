import sys
import types
from unittest.mock import MagicMock

import pytest

for mod_name in [
    "mcap",
    "mcap.reader",
    "mcap_ros2",
    "mcap_ros2.decoder",
    "rosbag2_py",
    "rclpy",
    "rclpy.serialization",
    "sensor_msgs",
    "sensor_msgs.msg",
    "trajectory_msgs",
    "trajectory_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "rosbag_recorder",
    "rosbag_recorder.msg",
]:
    if mod_name not in sys.modules:
        sys.modules[mod_name] = types.ModuleType(mod_name)

sys.modules["mcap.reader"].make_reader = MagicMock
sys.modules["mcap_ros2.decoder"].DecoderFactory = MagicMock
sys.modules["rosbag2_py"].SequentialReader = MagicMock
sys.modules["rosbag2_py"].StorageOptions = MagicMock
sys.modules["rosbag2_py"].ConverterOptions = MagicMock
sys.modules["rclpy.serialization"].deserialize_message = MagicMock
sys.modules["sensor_msgs.msg"].JointState = MagicMock
sys.modules["trajectory_msgs.msg"].JointTrajectory = MagicMock
sys.modules["nav_msgs.msg"].Odometry = MagicMock
sys.modules["geometry_msgs.msg"].Twist = MagicMock

from cyclo_data.converter.base_converter import (
    ConversionConfig,
    EpisodeData,
    RosbagToLerobotConverterBase,
)


def test_segment_video_discovery_rejects_legacy_renumbered_dir(tmp_path):
    bag_dir = tmp_path / "53"
    video_dir = bag_dir / "videos" / "153_0"
    video_dir.mkdir(parents=True)
    (bag_dir / "53_0.mcap").write_bytes(b"mcap")
    (video_dir / "cam_left_head.mp4").write_bytes(b"mp4")

    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test/repo", output_dir=tmp_path / "out")
    )

    with pytest.raises(FileNotFoundError, match="expected video segment directory"):
        converter._find_segment_video_files(bag_dir, "53_0")


def test_prepare_episodes_allows_mixed_positive_subtask_counts(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test/repo", output_dir=tmp_path / "out")
    )
    episodes = [
        EpisodeData(
            episode_index=0,
            length=3,
            subtask_segments=[{"subtask_index": 0}, {"subtask_index": 1}],
        ),
        EpisodeData(
            episode_index=1,
            length=3,
            subtask_segments=[
                {"subtask_index": 0},
                {"subtask_index": 1},
                {"subtask_index": 2},
            ],
        ),
    ]

    prepared = converter.prepare_episodes_for_writing(episodes)

    assert prepared == episodes


def test_prepare_episodes_rejects_mixed_single_and_subtask(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test/repo", output_dir=tmp_path / "out")
    )
    episodes = [
        EpisodeData(episode_index=0, length=3, subtask_segments=[]),
        EpisodeData(
            episode_index=1,
            length=3,
            subtask_segments=[{"subtask_index": 0}, {"subtask_index": 1}],
        ),
    ]

    assert converter.prepare_episodes_for_writing(episodes) == []
