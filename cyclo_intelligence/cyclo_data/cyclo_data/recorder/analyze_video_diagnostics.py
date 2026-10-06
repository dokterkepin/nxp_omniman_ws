"""Summarise VideoRecorder diagnostics parquet files.

Usage:
    python -m cyclo_data.recorder.analyze_video_diagnostics /path/to/episode
"""

from __future__ import annotations

import argparse
import json
from math import ceil, floor
from pathlib import Path
from typing import Iterable

import pyarrow.parquet as pq


def _diagnostics_files(paths: Iterable[Path]) -> list[Path]:
    files: list[Path] = []
    for path in paths:
        if path.is_file() and path.name.endswith("_diagnostics.parquet"):
            files.append(path)
            continue
        if path.is_dir():
            files.extend(path.glob("**/*_diagnostics.parquet"))
    return sorted(set(files))


def _stats_files(paths: Iterable[Path]) -> list[Path]:
    files: list[Path] = []
    for path in paths:
        if path.is_file() and path.name.endswith("_recorder_stats.json"):
            files.append(path)
            continue
        if path.is_dir():
            files.extend(path.glob("**/*_recorder_stats.json"))
    return sorted(set(files))


def _percentile(sorted_values: list[float], pct: float) -> float | None:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * pct / 100.0
    lo = floor(pos)
    hi = ceil(pos)
    if lo == hi:
        return sorted_values[int(pos)]
    frac = pos - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def _series_ms(columns: dict[str, list[int]], end: str, start: str) -> list[float]:
    return [
        (right - left) / 1_000_000.0
        for left, right in zip(columns[start], columns[end])
    ]


def _maybe_series_ms(
    columns: dict[str, list[int]],
    end: str,
    start: str,
) -> list[float]:
    if end not in columns or start not in columns:
        return []
    return _series_ms(columns, end, start)


def _first_existing(columns: dict[str, list[int]], names: list[str]) -> str | None:
    for name in names:
        if name in columns:
            return name
    return None


def _summarise(values: list[float]) -> str:
    if not values:
        return "n/a"
    sorted_values = sorted(values)
    return (
        f"p50={_percentile(sorted_values, 50):.3f} "
        f"p90={_percentile(sorted_values, 90):.3f} "
        f"p99={_percentile(sorted_values, 99):.3f} "
        f"max={max(sorted_values):.3f}"
    )


def _unique_flush_ms(columns: dict[str, list[int]]) -> list[float]:
    seen: set[tuple[int, int]] = set()
    values: list[float] = []
    for start, done in zip(
        columns["timestamp_flush_start_ns"],
        columns["timestamp_flush_done_ns"],
    ):
        key = (start, done)
        if key in seen:
            continue
        seen.add(key)
        values.append((done - start) / 1_000_000.0)
    return values


def _header_intervals_ms(columns: dict[str, list[int]]) -> list[float]:
    rows = sorted(zip(columns["frame_index"], columns["header_stamp_ns"]))
    return [
        (rows[i][1] - rows[i - 1][1]) / 1_000_000.0
        for i in range(1, len(rows))
    ]


def _read_columns(path: Path) -> dict[str, list[int]]:
    table = pq.read_table(path)
    return {
        name: table.column(name).to_pylist()
        for name in table.column_names
    }


def summarise_file(path: Path) -> list[tuple[str, str]]:
    columns = _read_columns(path)
    rows = len(columns.get("frame_index", []))
    queue_before = columns.get("queue_size_before", [])
    queue_after = columns.get("queue_size_after", [])
    metadata_queue_before = columns.get("metadata_queue_size_before", [])
    metadata_queue_after = columns.get("metadata_queue_size_after", [])
    frame_sizes = columns.get("frame_size_bytes", [])
    dequeue_col = _first_existing(columns, ["video_dequeue_ns", "dequeue_ns"])
    write_start_col = _first_existing(
        columns, ["raw_write_start_ns", "ffmpeg_write_start_ns"]
    )
    write_done_col = _first_existing(
        columns, ["raw_write_done_ns", "ffmpeg_write_done_ns"]
    )
    write_label = "raw write ms" if "raw_write_done_ns" in columns else "ffmpeg write ms"

    return [
        ("frames", str(rows)),
        ("header interval ms", _summarise(_header_intervals_ms(columns))),
        ("header -> callback ms", _summarise(
            _series_ms(columns, "callback_enter_ns", "header_stamp_ns")
        )),
        ("callback bytes copy ms", _summarise(
            _series_ms(columns, "bytes_copy_done_ns", "bytes_copy_start_ns")
        )),
        ("callback total ms", _summarise(
            _series_ms(columns, "enqueue_done_ns", "callback_enter_ns")
        )),
        ("enqueue ms", _summarise(
            _series_ms(columns, "enqueue_done_ns", "enqueue_start_ns")
        )),
        ("video queue wait ms", _summarise(
            _maybe_series_ms(columns, dequeue_col, "enqueue_done_ns")
            if dequeue_col else []
        )),
        ("raw queue wait ms", _summarise(
            _maybe_series_ms(columns, "raw_dequeue_ns", "video_dequeue_ns")
        )),
        (write_label, _summarise(
            _maybe_series_ms(columns, write_done_col, write_start_col)
            if write_done_col and write_start_col else []
        )),
        ("metadata enqueue ms", _summarise(
            _maybe_series_ms(
                columns,
                "metadata_enqueue_done_ns",
                "metadata_enqueue_start_ns",
            )
        )),
        ("metadata queue wait ms", _summarise(
            _maybe_series_ms(
                columns,
                "metadata_dequeue_ns",
                "metadata_enqueue_done_ns",
            )
        )),
        ("callback -> write done ms", _summarise(
            _maybe_series_ms(columns, write_done_col, "callback_enter_ns")
            if write_done_col else []
        )),
        ("header -> write done ms", _summarise(
            _maybe_series_ms(columns, write_done_col, "header_stamp_ns")
            if write_done_col else []
        )),
        ("timestamp flush ms", _summarise(_unique_flush_ms(columns))),
        ("queue size before", _summarise([float(value) for value in queue_before])),
        ("queue size after", _summarise([float(value) for value in queue_after])),
        ("metadata queue before", _summarise([
            float(value) for value in metadata_queue_before
        ])),
        ("metadata queue after", _summarise([
            float(value) for value in metadata_queue_after
        ])),
        ("frame size KB", _summarise([
            float(value) / 1024.0 for value in frame_sizes
        ])),
    ]


def summarise_stats_file(path: Path) -> list[tuple[str, str]]:
    data = json.loads(path.read_text(encoding="utf-8"))

    def _ms(name: str) -> str:
        value = data.get(name)
        if value is None:
            return "n/a"
        return f"{float(value) / 1_000_000.0:.3f}"

    return [
        ("frames received", str(data.get("frames_received", "n/a"))),
        ("frames written", str(data.get("frames_written", "n/a"))),
        ("frames metadata", str(data.get("frames_metadata_written", "n/a"))),
        ("frames remuxed", str(data.get("frames_remuxed", "n/a"))),
        ("drops queue", str(data.get("frames_dropped_queue", "n/a"))),
        ("drops invalid", str(data.get("frames_dropped_invalid", "n/a"))),
        ("pressure warnings", str(data.get("pressure_warning_count", "n/a"))),
        ("max callback items", str(data.get("max_callback_queue_items", "n/a"))),
        ("max raw items", str(data.get("max_raw_queue_items", "n/a"))),
        ("max metadata items", str(data.get("max_metadata_queue_items", "n/a"))),
        ("max callback MB", (
            f"{float(data.get('max_callback_queue_bytes', 0)) / 1024.0 / 1024.0:.3f}"
        )),
        ("max raw MB", (
            f"{float(data.get('max_raw_queue_bytes', 0)) / 1024.0 / 1024.0:.3f}"
        )),
        ("max enqueue ms", _ms("max_enqueue_wait_ns")),
        ("max raw write ms", _ms("max_raw_write_ns")),
        ("max metadata flush ms", _ms("max_metadata_flush_ns")),
        ("remux ms", _ms("remux_duration_ns")),
        ("raw write error", str(data.get("raw_write_error"))),
        ("metadata error", str(data.get("metadata_error"))),
        ("remux error", str(data.get("remux_error"))),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarise cyclo_data VideoRecorder diagnostics parquet files.",
    )
    parser.add_argument(
        "paths",
        nargs="+",
        type=Path,
        help="Episode directories or *_diagnostics.parquet files.",
    )
    args = parser.parse_args()

    files = _diagnostics_files(args.paths)
    stats_files = _stats_files(args.paths)
    if not files and not stats_files:
        print("No *_diagnostics.parquet or *_recorder_stats.json files found.")
        return 1

    for index, path in enumerate(stats_files):
        if index:
            print()
        print(path)
        for label, value in summarise_stats_file(path):
            print(f"  {label:24s} {value}")

    for index, path in enumerate(files):
        if index or stats_files:
            print()
        print(path)
        for label, value in summarise_file(path):
            print(f"  {label:24s} {value}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(0)
