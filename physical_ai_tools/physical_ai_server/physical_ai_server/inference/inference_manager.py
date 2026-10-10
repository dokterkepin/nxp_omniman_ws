#!/usr/bin/env python3
#
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

import hashlib
import os
import time

from lerobot.policies.pretrained import PreTrainedPolicy
import numpy as np
from physical_ai_server.utils.file_utils import read_json_file
import torch


class InferenceManager:

    def __init__(
            self,
            device: str = 'cuda'):

        self.device = device
        self.policy_type = None
        self.policy_path = None
        self.policy = None

    def validate_policy(self, policy_path: str) -> bool:
        result_message = ''
        if not os.path.exists(policy_path) or not os.path.isdir(policy_path):
            result_message = f'Policy path {policy_path} does not exist or is not a directory.'
            return False, result_message

        config_path = os.path.join(policy_path, 'config.json')
        if not os.path.exists(config_path):
            result_message = f'config.json file does not exist in {policy_path}.'
            return False, result_message

        config = read_json_file(config_path)
        if (config is None or
                ('type' not in config and 'model_type' not in config)):
            result_message = f'config.json malformed or missing fields in {policy_path}.'
            return False, result_message

        available_policies = self.__class__.get_available_policies()
        policy_type = config.get('type') or config.get('model_type')
        if policy_type not in available_policies:
            result_message = f'Policy type {policy_type} is not supported.'
            return False, result_message

        # Drop a cached policy when the requested checkpoint changes. The caller
        # only reloads when self.policy is None, and FINISH does not clear it -
        # so without this a second START_INFERENCE with a DIFFERENT policy_path
        # silently keeps running the first policy's weights under the new name.
        if self.policy is not None and self.policy_path != policy_path:
            self.clear_policy()

        self.policy_path = policy_path
        self.policy_type = policy_type
        return True, f'Policy {policy_type} is valid.'

    def load_policy(self):
        try:
            policy_cls = self._get_policy_class(self.policy_type)
            self.policy = policy_cls.from_pretrained(self.policy_path)
            return True
        except Exception as e:
            print(f'Failed to load policy from {self.policy_path}: {e}')
            return False

    def describe_policy(self):
        """What the server actually loaded, as text for the log: read from the
        loaded policy and the files it came from, not from the path it was
        asked for. The fingerprint comes from the weights in memory - two
        checkpoints with the same fingerprint are the same weights."""
        try:
            policy, path = self.policy, self.policy_path
            real = os.path.realpath(path)
            lines = [f'asked for  : {path}']
            if real != os.path.abspath(path):
                lines.append(f'resolved   : {real}')
            step = os.path.basename(os.path.dirname(real))
            if step.isdigit():
                lines.append(f'checkpoint : step {int(step)}')
            weights = os.path.join(real, 'model.safetensors')
            if os.path.exists(weights):
                info = os.stat(weights)
                saved = time.strftime('%Y-%m-%d %H:%M', time.localtime(info.st_mtime))
                lines.append(f'weights    : model.safetensors, {info.st_size / 1e6:.0f} MB, '
                             f'saved {saved}')
            n_params = sum(p.numel() for p in policy.parameters())
            lines.append(f'policy     : {type(policy).__name__}, {n_params / 1e6:.1f}M '
                         f'parameters, on {next(policy.parameters()).device}')
            config = policy.config
            settings = [f'{k} {getattr(config, k)}' for k in
                        ('chunk_size', 'n_action_steps', 'temporal_ensemble_coeff')
                        if hasattr(config, k)]
            lines.append(f'config     : {", ".join(settings)}')
            train = read_json_file(os.path.join(real, 'train_config.json')) or {}
            dataset = train.get('dataset') or {}
            if dataset or train.get('steps'):
                lines.append(f'trained on : {dataset.get("root") or dataset.get("repo_id")} '
                             f'({train.get("steps")} steps, batch {train.get("batch_size")})')
            digest = hashlib.sha256()
            with torch.no_grad():
                for name, tensor in sorted(policy.state_dict().items()):
                    digest.update(f'{name}:{tensor.double().sum().item():.6f}'.encode())
            lines.append(f'fingerprint: {digest.hexdigest()[:12]} (of the weights in memory)')
            return 'Policy loaded:\n  ' + '\n  '.join(lines)
        except Exception as e:
            return f'Policy loaded, but describing it failed: {e}'

    def clear_policy(self):
        if hasattr(self, 'policy'):
            del self.policy
            self.policy = None
        else:
            print('No policy to clear.')

    def get_policy_config(self):
        return self.policy.config

    def predict(
            self,
            images: dict[str, np.ndarray],
            state: list[float],
            task_instruction: str = None) -> list:

        observation = self._preprocess(images, state, task_instruction)
        with torch.inference_mode():
            action = self.policy.select_action(observation)
            action = action.squeeze(0).to('cpu').numpy()

        return action

    def _preprocess(
            self,
            images: dict[str, np.ndarray],
            state: list,
            task_instruction: str = None) -> dict:

        observation = self._convert_images2tensors(images)
        observation['observation.state'] = self._convert_np2tensors(state)
        for key in observation.keys():
            observation[key] = observation[key].to(self.device)

        if task_instruction is not None:
            observation['task'] = [task_instruction]

        return observation

    def _convert_images2tensors(
            self,
            images: dict[str, np.ndarray]) -> dict[str, torch.Tensor]:

        processed_images = {}
        for key, value in images.items():
            image = torch.from_numpy(value)
            image = image.to(torch.float32) / 255
            image = image.permute(2, 0, 1)
            image = image.to(self.device, non_blocking=True)
            image = image.unsqueeze(0)
            processed_images['observation.images.' + key] = image

        return processed_images

    def _convert_np2tensors(
            self,
            data):
        if isinstance(data, list):
            data = np.array(data)
        tensor_data = torch.from_numpy(data)
        tensor_data = tensor_data.to(torch.float32)
        tensor_data = tensor_data.to(self.device, non_blocking=True)
        tensor_data = tensor_data.unsqueeze(0)

        return tensor_data

    def _get_policy_class(self, name: str) -> PreTrainedPolicy:
        if name == 'tdmpc':
            from lerobot.policies.tdmpc.modeling_tdmpc import TDMPCPolicy

            return TDMPCPolicy
        elif name == 'diffusion':
            from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

            return DiffusionPolicy
        elif name == 'act':
            from lerobot.policies.act.modeling_act import ACTPolicy

            return ACTPolicy
        elif name == 'vqbet':
            from lerobot.policies.vqbet.modeling_vqbet import VQBeTPolicy

            return VQBeTPolicy
        elif name == 'pi0':
            from lerobot.policies.pi0.modeling_pi0 import PI0Policy

            return PI0Policy
        elif name == 'pi0fast':
            from lerobot.policies.pi0fast.modeling_pi0fast import PI0FASTPolicy
            return PI0FASTPolicy
        elif name == 'smolvla':
            from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
            return SmolVLAPolicy
        # TODO: Uncomment when GrootN1Policy is implemented
        # elif name == 'groot-n1':
        #     from Isaac.groot_n1.policies.groot_n1 import GrootN1Policy
        #     return GrootN1Policy
        else:
            raise NotImplementedError(
                f'Policy with name {name} is not implemented.')

    @staticmethod
    def get_available_policies() -> list[str]:
        return [
            'tdmpc',
            'diffusion',
            'act',
            'vqbet',
            'pi0',
            'pi0fast',
            'smolvla',
        ]

    @staticmethod
    def get_saved_policies():
        import os
        import json

        home_dir = os.path.expanduser('~')
        hub_dir = os.path.join(home_dir, '.cache/huggingface/hub')
        models_folder_list = [d for d in os.listdir(hub_dir) if d.startswith('models--')]

        saved_policy_path = []
        saved_policy_type = []

        for model_folder in models_folder_list:
            model_path = os.path.join(hub_dir, model_folder)
            snapshots_path = os.path.join(model_path, 'snapshots')

            # Check if snapshots directory exists
            if os.path.exists(snapshots_path) and os.path.isdir(snapshots_path):
                # Get list of folders inside snapshots directory
                snapshot_folders = [
                    d for d in os.listdir(snapshots_path)
                    if os.path.isdir(os.path.join(snapshots_path, d))
                ]

            # Check if pretrained_model folder exists in each snapshot folder
            for snapshot_folder in snapshot_folders:
                snapshot_path = os.path.join(snapshots_path, snapshot_folder)
                pretrained_model_path = os.path.join(snapshot_path, 'pretrained_model')

                # If pretrained_model folder exists, add to saved_policies
                if os.path.exists(pretrained_model_path) and os.path.isdir(pretrained_model_path):
                    config_path = os.path.join(pretrained_model_path, 'config.json')
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config = json.load(f)
                                if 'type' in config:
                                    saved_policy_path.append(pretrained_model_path)
                                    saved_policy_type.append(config['type'])
                                elif 'model_type' in config:
                                    saved_policy_path.append(pretrained_model_path)
                                    saved_policy_type.append(config['model_type'])
                        except (json.JSONDecodeError, IOError):
                            # If config.json cannot be read, store path only
                            print('File IO Errors : ', IOError)

        return saved_policy_path, saved_policy_type
