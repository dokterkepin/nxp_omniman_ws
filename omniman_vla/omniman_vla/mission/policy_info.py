"""Three lines about a policy checkpoint, read from its files - no torch, no
physical_ai_server needed, so a mission can log them when it starts a policy."""

import json
import math
import os
import struct


def _weights(path):
    """How many values model.safetensors holds, from its header (the file is not
    loaded). Counts every stored tensor, so a little more than the parameters."""
    with open(path, 'rb') as f:
        size = struct.unpack('<Q', f.read(8))[0]
        header = json.loads(f.read(size))
    header.pop('__metadata__', None)
    return sum(math.prod(t['shape']) for t in header.values())


def policy_info(policy_path):
    """[path, policy, config] lines for the checkpoint folder policy_path
    (the one with config.json and model.safetensors), links resolved."""
    if not policy_path:
        return ['path   : not set here - policy_runner.yaml\'s policy_path is used']
    real = os.path.realpath(os.path.expanduser(policy_path))
    lines = [f'path   : {real}']
    try:
        with open(os.path.join(real, 'config.json')) as f:
            config = json.load(f)
        policy = [str(config.get('type') or config.get('model_type'))]
        try:
            values = _weights(os.path.join(real, 'model.safetensors'))
            policy.append(f'{values / 1e6:.1f}M weights')
        except (OSError, ValueError, KeyError):
            pass
        if config.get('device'):
            policy.append(f'device {config["device"]}')
        lines.append(f'policy : {", ".join(policy)}')
        settings = [f'{k} {config[k]}' for k in
                    ('chunk_size', 'n_action_steps', 'temporal_ensemble_coeff') if k in config]
        lines.append(f'config : {", ".join(settings)}')
    except (OSError, ValueError) as e:
        lines.append(f'policy : cannot read config.json in it ({e})')
    return lines
