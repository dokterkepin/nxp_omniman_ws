"""
omniman_act - run an ACT policy the way physical_ai_server did, inside Cyclo.

Cyclo's own LeRobot engine (lerobot_engine) asks the policy for its whole
action chunk, policy.predict_action_chunk(), and its runtime plays that chunk
from a queue. LeRobot's ACT keeps its smoothing in a different method,
policy.select_action(): with temporal_ensemble_coeff set in the model's
config.json it predicts a chunk every step and returns ONE action, the average
of all earlier predictions for this step; otherwise it plays n_action_steps
from a queue. physical_ai_server called select_action on every tick; Cyclo
never does, so those two settings of the model have no effect there.

This engine is Cyclo's engine - observation, preprocessing, postprocessing,
loading, all inherited and unchanged - with one difference: each call returns
one action from select_action(), as a chunk of length 1.

What it needs from the runtime (the settings are Cyclo's own):
  - Action Request: sync or async both work. Each answer is one action, so
    the runtime's queue is always low and it asks once per step either way
    (measured: 30.0 asks/s in both modes at Dataset FPS 30).
  - POSTPROCESS_ACTIONS=false (set by services.py when this engine is chosen):
    no interpolation to 100 Hz, the loop runs at the model's rate - Dataset
    FPS, 30 for omniman_pick_and_place.

Lives next to lerobot_engine in cyclo_brain/policy/lerobot/, where the runtime
finds engines. Chosen in native/policy/lerobot_backend.yaml (engine: omniman_act),
then Restart on the LeRobot card.
"""

import logging
import time

from lerobot_engine.engine import LeRobotEngine

# A child of the SDK's logger, so the backend's INFO output shows it.
logger = logging.getLogger('zenoh_ros2_sdk.omniman_act')

# A pause longer than this between two requests means the run was stopped or
# paused (the runtime does not ask while paused): the policy forgets what it
# predicted before, as at the start of an episode.
RESET_AFTER_S = 1.0
# Asking much faster than the dataset rate is a wrong rate setting (Dataset FPS).
BURST_RATE_HZ = 45.0
REPORT_EVERY_S = 5.0


class OmnimanActEngine(LeRobotEngine):
    """Cyclo's LeRobotEngine, answering with policy.select_action()."""

    def __init__(self):
        super().__init__()
        self._last_call = None
        self._window = None          # [start, calls, seconds spent predicting]

    def _predict_chunk(self, batch):
        """One action, as a chunk of one: (1, 1, action_dim)."""
        assert self._policy is not None
        action = self._policy.select_action(batch)        # (1, action_dim)
        if action.dim() == 1:
            action = action.unsqueeze(0)
        return action.unsqueeze(1)

    def get_action_chunk(self, request):
        now = time.monotonic()
        if (self._last_call is not None and self._policy is not None
                and now - self._last_call > RESET_AFTER_S):
            self._policy.reset()
            logger.info(f'policy.reset(): {now - self._last_call:.1f}s since the last '
                        'request - a new run, the earlier predictions are forgotten')
        self._last_call = now
        start = time.monotonic()
        result = super().get_action_chunk(request)
        self._watch(start)
        return result

    def _watch(self, started):
        """Every few seconds: how often it is asked and how long a prediction takes."""
        now = time.monotonic()
        if self._window is None:
            self._window = [now, 0, 0.0]
        self._window[1] += 1
        self._window[2] += now - started
        elapsed = now - self._window[0]
        if elapsed >= REPORT_EVERY_S:
            calls, spent = self._window[1], self._window[2]
            rate = calls / elapsed
            logger.info(f'asked {rate:.1f} times/s, prediction {spent / calls * 1000:.0f} ms avg')
            if rate > BURST_RATE_HZ:
                logger.warning(f'asked {rate:.0f} times/s - faster than a dataset rate: '
                               'check Dataset FPS, or the ensemble runs too fast')
            self._window = [now, 0, 0.0]

    def cleanup(self):
        self._last_call = None
        self._window = None
        super().cleanup()


def create_engine():
    return OmnimanActEngine()
