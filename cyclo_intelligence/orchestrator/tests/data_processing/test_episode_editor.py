import json
from pathlib import Path

import yaml

from cyclo_data.editor.episode_editor import DataEditor


def _write_episode(task_dir: Path, episode_index: int, video_index=None) -> Path:
    video_index = episode_index if video_index is None else video_index
    episode_dir = task_dir / str(episode_index)
    segment_name = f"{episode_index}_0"
    video_segment_name = f"{video_index}_0"
    episode_dir.mkdir(parents=True)
    (episode_dir / f"{segment_name}.mcap").write_bytes(b"mcap")
    (episode_dir / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "relative_file_paths": [f"{segment_name}.mcap"],
                    "duration": {"nanoseconds": 1_000_000_000},
                    "files": [{"path": f"{segment_name}.mcap"}],
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (episode_dir / "episode_info.json").write_text(
        json.dumps(
            {
                "episode_index": episode_index,
                "video_segments": [
                    {
                        "mcap": f"{segment_name}.mcap",
                        "video_dir": f"videos/{video_segment_name}",
                        "cameras": ["cam_left_head"],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    video_dir = episode_dir / "videos" / video_segment_name
    video_dir.mkdir(parents=True)
    (video_dir / "cam_left_head.mp4").write_bytes(b"mp4")
    (video_dir / "cam_left_head_timestamps.parquet").write_bytes(b"parquet")
    return episode_dir


def test_merge_rosbag_task_folders_renumbers_video_segments(tmp_path):
    source = tmp_path / "source"
    output = tmp_path / "merged"
    _write_episode(source, 100)

    result = DataEditor().merge_rosbag_task_folders([source], output)

    assert result.total_episodes == 1
    episode = output / "0"
    assert (episode / "0_0.mcap").exists()
    assert (episode / "videos" / "0_0" / "cam_left_head.mp4").exists()

    metadata = yaml.safe_load((episode / "metadata.yaml").read_text())
    info = metadata["rosbag2_bagfile_information"]
    assert info["relative_file_paths"] == ["0_0.mcap"]
    assert info["files"][0]["path"] == "0_0.mcap"

    episode_info = json.loads((episode / "episode_info.json").read_text())
    assert episode_info["episode_index"] == 0
    assert episode_info["video_segments"][0]["mcap"] == "0_0.mcap"
    assert episode_info["video_segments"][0]["video_dir"] == "videos/0_0"


def test_delete_rosbag_episodes_compact_renumbers_video_segments(tmp_path):
    task_dir = tmp_path / "task"
    _write_episode(task_dir, 0)
    _write_episode(task_dir, 5)

    result = DataEditor().delete_rosbag_episodes(task_dir, [0], compact=True)

    assert result.deleted_count == 1
    assert result.remaining_count == 1
    episode = task_dir / "0"
    assert (episode / "0_0.mcap").exists()
    assert (episode / "videos" / "0_0" / "cam_left_head.mp4").exists()

    episode_info = json.loads((episode / "episode_info.json").read_text())
    assert episode_info["episode_index"] == 0
    assert episode_info["video_segments"][0]["mcap"] == "0_0.mcap"
    assert episode_info["video_segments"][0]["video_dir"] == "videos/0_0"
