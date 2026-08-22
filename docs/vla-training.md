# VLA Training (ACT) with physical_ai_tools

End-to-end workflow for teaching omniman a manipulation task by demonstration:
record teleoperated demos → train an ACT policy → run it autonomously.

Uses the `omniman_vla` package (7-joint arm+gripper JTC, no MoveIt/mecanum) together with
ROBOTIS `physical_ai_tools`. Demonstrations come from the XM430 leader arm via
`leader_ros2_control`.

---

## Prerequisites

Leader Follower Teleoperation should be working first — otherwise there is no way to demonstrate
the task, so no data collection and no training. See
**[leader-teleop.md](leader-teleop.md)** for the leader bringup, gravity compensation, and tuning.

### Making LeRobot importable

`pip install lerobot` pulls the latest upstream release, which does **not** match the version
ROBOTIS vendors. `physical_ai_tools` ships its own LeRobot as a submodule at
`../physical_ai_tools/lerobot`, built from source — that is the copy everything must import,
or the versions silently diverge.

There are **two ways** to make that copy importable, and they are alternatives — you only need
one:

**Option A — `PYTHONPATH` (used on the robot PC).** Add to `.bashrc`:
```bash
export PYTHONPATH=/home/dokterkepin/anaconda3/envs/lerobot_train/lib/python3.10/site-packages:/home/dokterkepin/workspaces/nxp_omniman_ws/src/physical_ai_tools/lerobot/src:$PYTHONPATH
```

**Option B — editable install (used on the remote training machine, §5).**
```bash
cd .../physical_ai_tools/lerobot
pip install -e .
```
This writes a pointer into that interpreter's `site-packages` so `import lerobot` resolves to
the source tree — no `PYTHONPATH` needed. Verify with
`python3 -c "import lerobot; print(lerobot.__file__)"`, Pin the `datasets` version. LeRobot requires `datasets>=2.19.0,<=3.6.0`. A newer 4.x release breaks dataset loading

Verify GPU:
```bash
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

---

## 1. Bring up the robots

Four terminals. Start the follower first, then the leader, then connect them.

**Terminal 1 — follower (omniman) + camera:**
```bash
ros2 launch omniman_vla nxp_omniman_vla_launch.py
```

**Terminal 2 — leader (gravity compensation):**
```bash
ros2 launch leader_ros2_control leader_gravity_launch.py
```

Move the leader by hand so its pose roughly matches the follower **before** connecting them.

**Terminal 3 — teleop relay (the "go" button):**
```bash
ros2 launch leader_ros2_control teleop_bridges_launch.py
```

The follower now mirrors the leader, gripper included. Keeping this separate is deliberate:
nothing moves until you explicitly connect the two arms.

**Terminal 4 — physical_ai_server stack:**
```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```

Starts `physical_ai_server` + `rosbridge` (:9090) + `web_video_server` (:8080) + bag recorder.

**Terminal 5 — web UI:**
```bash
cd ~/workspaces/nxp_omniman_ws/src/physical_ai_tools/physical_ai_manager
npm start          # http://localhost:3000
```

---

## 2. Record demonstrations

In the UI: **Record** tab, Robot Type `omniman`.

| Field | Value |
|---|---|
| Task Name | e.g. `pick_and_place` — **keep identical across sessions** |
| Task Instruction | e.g. `pick the yellow object and place it on black box` |
| FPS | 30 |
| Warmup / Episode / Reset Time (s) | 5 / 20 / 5 |
| Num Episodes | 10 per session |
| Optimized Save | Enabled |
| Record Rosbag2 | Disabled |

Press **Start**. Each episode runs `Warmup → Recording → Reset`.

**Controls:** `Stop` saves the episode · `Retry` discards and redoes it · `Next` saves and skips
ahead · `Finish` ends the task.

> Same Task Name **appends** to the existing dataset — record in batches of ~10 across sessions
> rather than 50 in one sitting. Operator fatigue shows up in the data as progressively slower
> demos.

### Where the data lands

```
~/.cache/huggingface/lerobot/<user>/<task_name>/
├── data/chunk-000/episode_NNNNNN.parquet     # action + state
├── videos/chunk-000/observation.images.cam/  # camera, as .mp4
└── meta/{info,episodes,episodes_stats,tasks}.jsonl
```

Check episode count:
```bash
wc -l ~/.cache/huggingface/lerobot/<user>/<task_name>/meta/episodes.jsonl
```

Inspect episodes visually (same UI as the HF `visualize_dataset` Space, run locally):
```bash
cd ~/workspaces/nxp_omniman_ws/src/physical_ai_tools/lerobot
PYTHONPATH=src python3 -m lerobot.scripts.visualize_dataset_html \
    --repo-id <user>/<task_name> --port 9091
```
> Override `--port` — the default 9090 collides with rosbridge.

Delete bad episodes via **Data Tools → Delete** (accepts `0,1,5-9`). Don't delete parquet files
by hand; the metadata has to stay consistent.

### Changing the save location

The dataset root comes from `physical_ai_server.py`'s `DEFAULT_SAVE_ROOT_PATH`, overridable with
an env var (export it in the terminal that launches the server):

```bash
export PHYSICAL_AI_DATASET_ROOT=/home/dokterkepin/dataset
export HF_LEROBOT_HOME=/home/dokterkepin/dataset
```

> This only works because `data_manager.py` passes `root=self._save_path` to
> `LeRobotDatasetWrapper.create()`. Upstream omits that argument, so LeRobot falls back to
> `HF_LEROBOT_HOME` and writes to `~/.cache/...` regardless of what the server was configured
> with — and `HF_LEROBOT_HOME` alone can't fix it either, because saving runs in a spawned child
> process that doesn't inherit the parent's environment. If datasets ever reappear in `~/.cache`
> after a submodule update, that one-line `root=` is what got reverted.

### Pre-filling the UI fields

Recording defaults live in `physical_ai_manager/src/features/tasks/taskSlice.js`, under
`initialState.taskInfo`. The UI has no session persistence, so a browser refresh resets every
field to these values — setting them avoids retyping (and mistyping) the task name each time:

File-browser start directories are in `physical_ai_manager/src/constants/paths.js`, which
defaults to the Docker image's `/root/...` paths. Override them in `physical_ai_manager/.env`
(these only affect the browser UI, never where data is written):

```bash
REACT_APP_POLICY_MODEL_PATH=/home/dokterkepin/output/
REACT_APP_DOT_CACHE_PATH=/home/dokterkepin/.cache
```

> `.env` is read only at startup — restart `npm start`. Edits to `taskSlice.js` hot-reload.

---

## 3. Run inference

**Shut down the leader first.**

```
Terminal 2 (leader_gravity_launch.py)   → Ctrl+C   ← STOP
Terminal 3 (teleop_bridges_launch.py)   → LEAVE RUNNING
Terminal 1 (follower + camera)          → LEAVE RUNNING
Terminal 4 (physical_ai_server)         → LEAVE RUNNING
```

Confirm exactly one publisher:
```bash
ros2 topic info /leader/joint_trajectory
```

Then in the **Inference** tab: select the policy path
(`checkpoints/100000/pretrained_model`), enter the same task instruction, and press Start.

> **First autonomous run:** hand on the e-stop, workspace clear, nothing fragile in reach.

---

## 4. Train on a different machine — no physical_ai_tools needed

Training is plain PyTorch reading a folder of parquet/mp4 files — it never touches ROS,
`physical_ai_server`, or the web UI. That means it can run on **any machine with a GPU**,
independent of this project's ROS distro

**Install PyTorch via pip, in conda environment** — im using miniconda to setup my environment, the workflow we should do is exactly the same in the first time we clone the physical_ai_tools and install all the dependencies. after that we can install torch because torch is packed with cuda
```bash
pip3 install torch torchvision
```

verify:
```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.device_count())"
```

Then get the dataset and LeRobot source onto the remote machine and install the rest:
```bash
cd ~/lerobot
pip install -e .
```

### 4.2 The training command
```bash
cat > ~/train_cmd.sh << 'EOF'
python3 -m lerobot.scripts.train \
    --policy.type=act \
    --policy.push_to_hub=false \
    --dataset.repo_id=dokterkepin/omniman_pick_and_place \
    --dataset.root=/home/dokterkepin/dataset/omniman_pick_and_place \
    --dataset.image_transforms.enable=true \
    --batch_size=16 \
    --num_workers=12 \
    --steps=100000 \
    --save_freq=10000 \
    --output_dir=/home/dokterkepin/output/omniman_pick_and_place \
    --wandb.enable=true \
    --wandb.project=lerobot_train
EOF
bash ~/train_cmd.sh
```
