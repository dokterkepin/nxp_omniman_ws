#!/usr/bin/env python3
"""Drop trailing observation.state dims (e.g. base odometry) from a LeRobot v2.1 dataset.

WHY
    physical_ai_server records observation.state and action from the same
    joint_list, so they are always the same width - the base's odometry
    (follower_mobile:/mecanum_drive_controller/odometry) has to stay in
    joint_topic_list at record time, or recording crashes:
        ValueError: observation.state of shape (7,) does not match (10,)

    But once it's in observation.state, measured base velocity tracks the
    commanded velocity almost exactly within one frame, so the network can
    just echo it back instead of looking at the camera - and on the robot
    that echo is its own last command, which is why a base-correction policy
    trained on it gets stuck repeating whatever it just did.

    Record WITH the odometry topic present (as physical_ai_server requires),
    then run this script to remove it from observation.state before training.
    action is left untouched - it's the label, not the shortcut.

    At inference, comment out follower_mobile in omniman_config.yaml so the
    live state is the same width as what the trimmed policy was trained on.
    Uncomment it again before the next recording session.

The source dataset is never modified; this writes a new one. videos/ is
hard-linked (no extra disk, survives the source being renamed or deleted).

Usage (run in the lerobot_jazzy env):
    python trim_state.py SRC_DATASET DST_DATASET [--keep N]

    python trim_state.py omniman_base_correct_v5 omniman_base_correct_v6

Relative dataset names resolve against --root (default: ~/dataset/dokterkepin).
"""
import argparse
import json
import os
import shutil
import sys
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

COL = 'observation.state'
DEFAULT_ROOT = Path.home() / 'dataset' / 'dokterkepin'


def resolve(p, root):
    p = Path(p).expanduser()
    return p if p.is_absolute() else root / p


def link_tree(src, dst):
    """Hard-link every file under src into dst; copy if links aren't possible
    (e.g. across filesystems)."""
    for r, _, files in os.walk(src):
        out = dst / Path(r).relative_to(src)
        out.mkdir(parents=True, exist_ok=True)
        for f in files:
            try:
                os.link(Path(r) / f, out / f)
            except OSError:
                shutil.copy2(Path(r) / f, out / f)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--keep', type=int, default=7,
                    help='number of leading state dims to keep (default 7: arm + gripper)')
    ap.add_argument('--root', default=str(DEFAULT_ROOT),
                    help=f'base dir for relative dataset names (default: {DEFAULT_ROOT})')
    args = ap.parse_args()

    root = Path(args.root).expanduser()
    src, dst, keep = resolve(args.src, root), resolve(args.dst, root), args.keep

    info_path = src / 'meta' / 'info.json'
    if not info_path.exists():
        sys.exit(f'not a dataset: {info_path} missing')
    if dst.exists():
        sys.exit(f'{dst} already exists - pick another name or remove it first')

    info = json.loads(info_path.read_text())
    if info.get('codebase_version') != 'v2.1':
        sys.exit(f'expected codebase v2.1, got {info.get("codebase_version")}')
    width = info['features'][COL]['shape'][0]
    if width <= keep:
        sys.exit(f'{COL} is already {width} wide - nothing to trim (already trimmed?)')

    # ---- meta ------------------------------------------------------------
    dst.mkdir(parents=True)
    shutil.copytree(src / 'meta', dst / 'meta')

    feat = info['features'][COL]
    dropped = feat['names'][keep:]
    feat['shape'] = [keep]
    feat['names'] = feat['names'][:keep]
    (dst / 'meta' / 'info.json').write_text(json.dumps(info, indent=4))

    stats_path = dst / 'meta' / 'episodes_stats.jsonl'
    out_lines = []
    for line in stats_path.read_text().splitlines():
        d = json.loads(line)
        st = d['stats'][COL]
        for k in ('min', 'max', 'mean', 'std'):
            st[k] = st[k][:keep]
        out_lines.append(json.dumps(d))
    stats_path.write_text('\n'.join(out_lines) + '\n')

    # ---- videos ------------------------------------------------------------
    if (src / 'videos').exists():
        link_tree((src / 'videos').resolve(), dst / 'videos')

    # ---- data --------------------------------------------------------------
    n = 0
    for src_pq in sorted((src / 'data').rglob('*.parquet')):
        tbl = pq.read_table(src_pq)
        old = tbl.column(COL)
        rows = [None if v is None else v[:keep] for v in old.to_pylist()]
        t = old.type
        inner = t.value_type if hasattr(t, 'value_type') else pa.float32()
        new_t = (pa.list_(inner, keep) if pa.types.is_fixed_size_list(t)
                 else pa.list_(inner))
        tbl = tbl.set_column(tbl.schema.get_field_index(COL), COL,
                             pa.array(rows, type=new_t))
        dst_pq = dst / src_pq.relative_to(src)
        dst_pq.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(tbl, dst_pq)
        n += 1

    print(f'{n} episodes  {src.name} -> {dst.name}')
    print(f'{COL}: {width} -> {keep} dims   dropped {dropped}')
    print('action: unchanged')


if __name__ == '__main__':
    main()
