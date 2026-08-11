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

you an install lerobot framework by using ```bash pip install lerobot``` but robotis use different and lower version of lerobot framework (it does build from sorce instead of binary install)
at the first time you clone the repository, it will located in physical_ai_tools package [../physical_ai_tools/lerobot].
the lerobot framework should match, therefore it is better to source the cloned version by write in .bashrc:
```bash
export PYTHONPATH=/home/dokterkepin/workspaces/nxp_omniman_ws/src/physical_ai_tools/lerobot/src:$PYTHONPATH
```


**Pin the `datasets` version.** LeRobot requires `datasets>=2.19.0,<=3.6.0`. A newer 4.x
release breaks dataset loading:

```bash
pip3 install 'datasets<=3.6.0'
```

> **Why this matters:** with `datasets` 4.x the UI fails with
> `Invalid repository name, Please change the repository name`. That message is misleading —
> the repo name is fine. `physical_ai_server` catches *any* dataset exception and reports it
> as a name error. The real error appears in the server terminal as
> `Error checking lerobot dataset: ...`. Telltale sign: creating a new dataset works, but
> appending to an existing one always fails.
>
> If you already recorded under 4.x, that data is unreadable (`Feature type 'List' not found`)
> and must be re-recorded. After changing the version you **must restart `physical_ai_server`** —
> a running process keeps the old library in memory.

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

---

## 3. Train

**Training** tab. Select the dataset, then:

| Field | Value |
|---|---|
| Policy | `act` |
| Device | `cuda` |
| Batch Size | 8 (use 4 if OOM, or with 2+ cameras) |
| Steps | 100000 |
| Num Workers | 4 |
| Save Frequency | 10000 |
| Seed | 1000 |

On an RTX 4060 Laptop (8 GB): ~0.176 s/step → **~5 hours** for 100k steps, 52M parameters.

Checkpoints land in:
```
physical_ai_tools/lerobot/outputs/train/<output_name>/checkpoints/NNNNNN/pretrained_model/
```

---

## 4. Run inference

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

## 5. Train on a different machine — no physical_ai_tools needed

Training is plain PyTorch reading a folder of parquet/mp4 files — it never touches ROS,
`physical_ai_server`, or the web UI. That means it can run on **any machine with a GPU**,
independent of this project's ROS distro

**Install PyTorch via pip, in conda environment** — im using miniconda to setup my environment, the workflow we should do is exactly the same in the first time we clone the physical_ai_tools and install all the dependencies. after that we can install torch because torch is packed with cuda
```bash
pip3 install torch torchvision
```

Then get the dataset and LeRobot source onto the remote machine and install the rest:
```
cd ~/lerobot
pip install -e .
pip install 'datasets<=3.6.0'   # same version that used by physical_ai_tools 
```

### 5.2 The training command
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

### 5.3 Live monitoring with Weights & Biases (optional)
The UI's live loss graph comes from `physical_ai_server` always setting `wandb.enable: false`
internally — calling the CLI directly, you can turn it on for the same kind of live browser
dashboard:

```bash
pip install wandb
wandb login          # free account, one-time
```
Add `--wandb.enable=true --wandb.project=<name>` (already in the command above). The run prints
a `https://wandb.ai/...` URL — open it for live `train/loss`, `train/lr`, GPU utilization, etc.

> This uploads metrics to a third-party cloud service by default. Use `--wandb.mode=offline` to
> log locally instead, at the cost of the live browser view.
>
> The `eval/*` panels seen in LeRobot's own example dashboards (success rate, reward, video)
> only populate for datasets trained against a simulated `env` — real-robot runs like this one
> have `env: None`, so those panels simply never appear here. Not a bug.