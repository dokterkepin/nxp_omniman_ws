#!/usr/bin/env python3
"""Check a dataset and, optionally, a trained checkpoint against it - offline, no robot.

DATASET PART (always)
    How often each base axis moves, whether axes move together, and which
    values appear. Rotation that is rare, or never overlaps with driving, is
    what made ACT learn "never rotate".

POLICY PART (with --ckpt)
    Feeds frames the policy was trained on and compares its predicted base
    velocity with what you actually commanded. If it can't reproduce your own
    training data, it will not work on the robot either, so this is the check
    to run before inference.

        you |0.524|  policy |0.0001|   -> never learned that axis
        right direction 97%            -> learned it (50% = coin flip)

Usage (run from any directory, in the lerobot_jazzy env):
    python check_policy.py omniman_base_correct_v4
    python check_policy.py omniman_base_correct_v4 \
        --ckpt ~/output/omniman_base_correct_v4/checkpoints/100000/pretrained_model
    ... --device cpu      # leave the GPU to a running training job
    ... --zero-vel        # zero the base velocity in the input = robot at rest

Relative dataset names resolve next to this script (dataset/dokterkepin/).
"""
import argparse
import glob
import sys
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

HERE = Path(__file__).resolve().parent
LEROBOT_SRC = Path.home() / 'workspaces/nxp_omniman_ws/src/physical_ai_tools/lerobot/src'
BASE = slice(7, 10)       # action dims: linear_x, linear_y, angular_z
EPS = 1e-4


def dataset_report(root):
    files = sorted(glob.glob(str(root / 'data' / '*' / '*.parquet')))
    a = np.concatenate([np.stack(pq.read_table(f).column('action').to_pylist()) for f in files])
    b = a[:, BASE]
    act = np.abs(b) > EPS
    moving = act.any(1)
    print(f'== dataset {root.name}: {len(files)} episodes, {len(a)} frames')
    print(f'   frames with any base motion: {moving.mean() * 100:.1f}%')
    for i, n in enumerate(('linear_x', 'linear_y', 'angular_z')):
        vals = sorted(set(np.round(b[act[:, i], i], 3)))
        shown = vals if len(vals) <= 6 else f'{len(vals)} distinct (continuous)'
        print(f'   {n:10} active {act[:, i].mean() * 100:5.1f}%   values {shown}')
    if moving.any():
        k = act[moving].sum(1)
        print(f'moving frames with 1 axis {np.mean(k == 1) * 100:.1f}%, 2+ axes {np.mean(k >= 2)
                                                                                 * 100:.1f}%')
        print(f'(forward+rotate together {np.mean(act[moving, 0]
                                                  & act[moving, 2]) * 100:.1f}%)')


def policy_report(root, ckpt, device, stride, zero_vel):
    sys.path.insert(0, str(LEROBOT_SRC))
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import get_policy_class

    cfg = PreTrainedConfig.from_pretrained(ckpt)
    cfg.device = device
    pol = get_policy_class(cfg.type).from_pretrained(ckpt, config=cfg).to(device).eval()
    want = cfg.input_features['observation.state'].shape[0]

    ds = LeRobotDataset(f'local/{root.name}', root=root)
    task = ds.meta.tasks[0] if isinstance(ds.meta.tasks, dict) else ds.meta.tasks.index[0]
    have = ds.meta.features['observation.state']['shape'][0]
    print(f'\n== policy {cfg.type} from {ckpt}')
    print(f'   state: checkpoint expects {want}, dataset has {have}'
          + ('  -> using the first %d' % want if have > want else ''))
    if have < want:
        sys.exit('   dataset state is narrower than the checkpoint expects - wrong dataset?')

    names = ('linear_x', 'linear_y', 'angular_z')
    gt, pr = [], []
    for i in range(0, len(ds), stride):
        d = ds[i]
        state = d['observation.state'][:want].clone()
        if zero_vel and want >= 10:
            state[BASE] = 0
        batch = {k: d[k][None].to(device) for k in cfg.image_features}
        batch['observation.state'] = state[None].to(device)
        batch['task'] = [task]
        with torch.no_grad():
            out = pol.predict_action_chunk(batch)[0, 0].float().cpu().numpy()
        gt.append(d['action'][BASE].numpy())
        pr.append(out[BASE])
    g, p = np.array(gt), np.array(pr)
    print(f'   {len(g)} frames checked (every {stride}th)' +
          ('   [base velocity input zeroed]' if zero_vel else ''))
    for i, n in enumerate(names):
        m = np.abs(g[:, i]) > EPS
        if m.sum() == 0:
            continue
        you, got = np.abs(g[m, i]).mean(), np.abs(p[m, i]).mean()
        # The sign of a near-zero output is noise, so don't report a
        # "right direction" percentage for an axis that was never learned.
        if got < 0.05 * you:
            verdict = 'output ~0 -> NOT learned'
        else:
            verdict = f'right direction {(np.sign(p[m, i])
                                          == np.sign(g[m, i])).mean() * 100:3.0f}%'
        print(f'   {n:10} when you moved ({m.sum():4d} frames): you |{you:.3f}|'
              f'  policy |{got:.4f}| max {np.abs(p[m, i]).max():.4f}   {verdict}')
    still = ~(np.abs(g) > EPS).any(1)
    if still.any():
        # "Moving" = at least half the speed you typically used on that axis;
        # axes you never used are ignored rather than judged on noise.
        typical = np.array([np.abs(g[np.abs(g[:, j]) > EPS, j]).mean() if (np.abs(g[:, j])
                                                                           > EPS).any()
                            else np.inf for j in range(3)])
        moves = (np.abs(p[still]) > 0.5 * typical).any(1)
        print(f'   when you stood still ({still.sum()} frames): '
              f'policy still moves in {100 * moves.mean():.1f}% of them')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--ckpt', help='.../checkpoints/<step>/pretrained_model')
    ap.add_argument('--device', default=None, help='cuda or cpu (default: cuda if available)')
    ap.add_argument('--stride', type=int, default=10, help='check every Nth frame (default 10)')
    ap.add_argument('--zero-vel', action='store_true', help='zero base velocity in the input')
    args = ap.parse_args()

    root = Path(args.dataset).expanduser()
    root = root if root.is_absolute() else HERE / root
    if not (root / 'meta' / 'info.json').exists():
        sys.exit(f'not a dataset: {root}')

    dataset_report(root)
    if args.ckpt:
        import torch
        device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
        policy_report(root, str(Path(args.ckpt).expanduser()), device, args.stride, args.zero_vel)


if __name__ == '__main__':
    main()
