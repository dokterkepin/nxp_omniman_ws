# Copyright 2025 ROBOTIS CO., LTD.
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
# Author: Dongyun Kim

"""ROSbag file reader for extracting messages and metadata.

Uses the mcap Python library directly to avoid rosbag2_py compatibility
issues (e.g., type_description_hash version mismatch).
"""

from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


class BagReader:
    """
    ROSbag file reader for extracting messages.

    Reads MCAP files using the mcap Python library with mcap_ros2 decoder.
    """

    def __init__(self, bag_path: Path, logger=None):
        self.bag_path = Path(bag_path)
        self.logger = logger
        self._topic_type_map: Dict[str, str] = {}
        self._mcap_file: Optional[str] = None
        self._mcap_files: List[Path] = []
        self._decoder_factory = DecoderFactory()

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def _find_mcap_files(self) -> List[Path]:
        """Find the MCAP file(s) to read."""
        # If bag_path is a file, use it directly
        if self.bag_path.is_file() and self.bag_path.suffix == '.mcap':
            return [self.bag_path]

        # If bag_path is a directory, find the MCAP file
        if self.bag_path.is_dir():
            # Prefer episode.mcap (MP4 converter output)
            episode_mcap = self.bag_path / 'episode.mcap'
            if episode_mcap.exists():
                return [episode_mcap]

            # Fall back to all .mcap files. ROSbag2 can split one logical
            # recording across multiple files; subtask full-episode folders
            # also keep one MCAP per subtask (for example 0_0, 0_1, 0_2).
            mcap_files = sorted(self.bag_path.glob('*.mcap'))
            if mcap_files:
                return mcap_files

        return []

    def open(self) -> bool:
        """Open the bag file for reading. Returns True if successful."""
        self._mcap_files = self._find_mcap_files()
        self._mcap_file = str(self._mcap_files[0]) if self._mcap_files else None
        if not self._mcap_files:
            self._log_error(f"No MCAP file found in: {self.bag_path}")
            return False

        try:
            # Read topic types from each summary. Split MCAPs from one logical
            # episode usually share topics, but unioning keeps this robust.
            for mcap_file in self._mcap_files:
                with open(mcap_file, 'rb') as f:
                    reader = make_reader(f, decoder_factories=[self._decoder_factory])
                    summary = reader.get_summary()
                    if summary is None:
                        self._log_error(f"Failed to read MCAP summary: {mcap_file}")
                        return False

                    for channel_id, channel in summary.channels.items():
                        schema_id = channel.schema_id
                        schema = summary.schemas.get(schema_id)
                        schema_name = schema.name if schema else "unknown"
                        self._topic_type_map[channel.topic] = schema_name

            self._log_info(
                f"Opened {len(self._mcap_files)} MCAP file(s): "
                f"{', '.join(path.name for path in self._mcap_files)} "
                f"({len(self._topic_type_map)} topics)"
            )
            return True
        except Exception as e:
            self._log_error(f"Failed to open MCAP file: {e}")
            return False

    def get_topic_types(self) -> Dict[str, str]:
        """Get mapping of topic names to their message types."""
        return self._topic_type_map.copy()

    def read_messages(
        self, topic_filter: Optional[List[str]] = None
    ) -> Iterator[Tuple[str, Any, float]]:
        """
        Read messages from the bag file.

        Args:
            topic_filter: Optional list of topic names. Only these topics
                will be decoded, significantly improving performance.

        Yields:
            Tuple of (topic_name, deserialized_message, timestamp_sec)
        """
        if not self._mcap_files:
            self._log_error("MCAP file not opened. Call open() first.")
            return

        topic_set = set(topic_filter) if topic_filter else None

        for mcap_file in self._mcap_files:
            with open(mcap_file, 'rb') as f:
                reader = make_reader(f, decoder_factories=[self._decoder_factory])
                for schema, channel, message, decoded_msg in reader.iter_decoded_messages(
                    topics=topic_filter
                ):
                    if topic_set and channel.topic not in topic_set:
                        continue
                    timestamp_sec = message.log_time / 1e9
                    if decoded_msg is not None:
                        yield channel.topic, decoded_msg, timestamp_sec

    def read_raw_messages(
        self, topic_filter: Optional[List[str]] = None
    ) -> Iterator[Tuple[str, bytes, float, str]]:
        """
        Read raw (serialized) messages from the bag file.

        Yields:
            Tuple of (topic_name, raw_data, timestamp_sec, topic_type)
        """
        if not self._mcap_files:
            self._log_error("MCAP file not opened. Call open() first.")
            return

        topic_set = set(topic_filter) if topic_filter else None

        for mcap_file in self._mcap_files:
            with open(mcap_file, 'rb') as f:
                reader = make_reader(f)
                for schema, channel, message in reader.iter_messages(
                    topics=topic_filter
                ):
                    if topic_set and channel.topic not in topic_set:
                        continue
                    timestamp_sec = message.log_time / 1e9
                    topic_type = self._topic_type_map.get(channel.topic, "")
                    yield channel.topic, message.data, timestamp_sec, topic_type

    def get_time_range(self) -> Tuple[float, float]:
        """Get the time range of the bag file."""
        min_time = float("inf")
        max_time = float("-inf")

        if not self._mcap_files:
            return (0.0, 0.0)

        for _, _, timestamp_sec, _ in self.read_raw_messages():
            min_time = min(min_time, timestamp_sec)
            max_time = max(max_time, timestamp_sec)

        if min_time == float("inf"):
            return (0.0, 0.0)

        return (min_time, max_time)

    def close(self):
        """Close the bag reader."""
        self._mcap_file = None
        self._mcap_files = []
        self._topic_type_map = {}

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
