# cyclo_brain

Everything related to policy training and inference lives under this folder.

`cyclo_brain` intentionally uses the old shared policy runtime architecture:
each backend container runs the same two process layout, and backend-specific
code only implements model loading, preprocessing, prediction, and cleanup.

- `sdk/` contains host- and container-shared Python assets such as
  `robot_client`, `action_chunk_processing`, and `zenoh_ros2_sdk`.
- `policy/common/runtime/` contains the shared Main Runtime and Engine Process.
- `policy/<backend>/<backend>_engine/` contains backend-specific policy code.
- `policy/common/s6-services/` provides the shared `main-runtime` and
  `engine-process` longruns used by LeRobot and GR00T.

See [`STRUCTURE.md`](STRUCTURE.md) and
[`docs/architecture.html`](docs/architecture.html) for the full architecture.

## Runtime Contract

Every backend follows the same service shape:

1. The UI, orchestrator, BT, or CLI calls `/<backend>/inference_command`.
2. The Main Runtime owns session state, action buffering, command publishing,
   and the control loop.
3. The Engine Process owns model loading, observation reads, and model
   inference.
4. The Engine Process returns an `action_list` shaped `(T, D)`.
5. The Main Runtime buffers the action list and pops at the control cadence.

The Engine Process never publishes robot commands. Robot command output always
goes through the Main Runtime and `RobotClient`.

## Safety Modes

Inference commands include a robot-publish gate:

- `publish_to_robot = false`: simulation / dry-run mode. The runtime publishes
  `/inference/trajectory_preview` for the 3D viewer and does not publish robot
  command topics.
- `publish_to_robot = true`: robot mode. The runtime publishes the same 3D
  preview and also publishes configured robot command topics.

When the action buffer is empty, `ActionChunkProcessor.pop_action()` returns
`None`; the control loop does not repeat the previous action. Pause, stop,
unload, and output-mode changes clear buffered actions so a stale command cannot
jump into the robot after a mode switch.

## Backend Layout

```text
cyclo_brain/
├── sdk/
│   ├── action_chunk_processing/
│   ├── robot_client/
│   └── zenoh_ros2_sdk/
└── policy/
    ├── common/
    │   ├── runtime/
    │   │   ├── main_runtime/
    │   │   └── engine_process/
    │   └── s6-services/
    │       ├── main-runtime/
    │       └── engine-process/
    ├── lerobot/
    │   ├── Dockerfile.{arm64,amd64}
    │   ├── lerobot/
    │   └── lerobot_engine/
    └── groot/
        ├── Dockerfile.{arm64,amd64}
        ├── Isaac-GR00T/
        └── groot_engine/
```

## Adding A Backend

1. Add `policy/<backend>/<backend>_engine/`.
2. Implement the shared `InferenceEngine` contract:
   `load_policy`, `get_action_chunk`, `cleanup`, and `is_ready`.
3. Use `policy/common/runtime` and `policy/common/s6-services`; do not create
   backend-specific runtime processes unless the architecture itself changes.
4. Add the backend service to `docker/docker-compose.yml` and
   `docker/supervisor_api/app.py`.
5. If runtime structure changes, update `STRUCTURE.md` and
   `docs/architecture.html` in the same change.
