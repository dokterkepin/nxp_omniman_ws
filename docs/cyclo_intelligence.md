# Cyclo Intelligence: record, train and run an ACT policy

End-to-end workflow for teaching omniman a manipulation task by demonstration, with ROBOTIS
Cyclo Intelligence 1.4.0 (`cyclo_intelligence/`, LeRobot 0.6): record teleoperated demos → train
an ACT policy → run it autonomously.

Cyclo runs natively here, with `ros2 launch` and no Docker. It replaces physical_ai_tools on the
`cyclo_adapt` branch; for physical_ai_tools, use the `jazzy` branch and
[physical_ai.md](physical_ai.md). The workflow is the same; the tools differ.

Uses the `omniman_vla` package (7-joint arm+gripper JTC) together with Cyclo. Demonstrations come
from the XM430 leader arm via `leader_ros2_control`.

---

## Prerequisites

Leader Follower Teleoperation should be working first — otherwise there is no way to demonstrate
the task, so no data collection and no training. See
**[leader-teleop.md](leader-teleop.md)** for the leader bringup, gravity compensation, and tuning.

The workspace is cloned and builds as in [README.md](../README.md), on ROS 2 Jazzy, and the PC
has an NVIDIA GPU with its driver. Do the install on the PC that runs Cyclo (UI, recording,
policy, training), not on the robot.

### Install (once per PC)

**1. Switch to the Cyclo branch**
```bash
cd ~/workspaces/nxp_omniman_ws/src
git checkout cyclo_adapt
```

**2. Packages Cyclo needs that rosdep does not install**
```bash
sudo apt install python3-venv ros-jazzy-rmw-zenoh-cpp ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server ros-jazzy-rosbag2-storage-mcap ros-jazzy-nav2-msgs
```

**3. Node.js 22 and Miniconda** — the web UI is built with Node 22 or newer; the policy backend
runs in a conda env, and the installer looks for conda in `~/miniconda3`.
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 22

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p ~/miniconda3
```

**4. Use Zenoh** — Cyclo talks to ROS over Zenoh. Add to `~/.bashrc`, then open a new terminal:
```bash
export ROS_DOMAIN_ID=67                    # the same as the robot
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

**5. Build**
```bash
cd ~/workspaces/nxp_omniman_ws
colcon build --symlink-install
```
Keep `--symlink-install`: omniman's behaviour-tree nodes are in subfolders that Cyclo's
`setup.py` does not list, and only this build finds them.

**6. Run the installer** — it takes a while, and it is safe to run again:
```bash
src/cyclo_intelligence/native/install.sh
```

| Where | What |
|---|---|
| `nxp_omniman_ws/cyclo/venv` | Python packages for Cyclo's ROS nodes |
| `nxp_omniman_ws/cyclo/workspace` | Cyclo's data: recordings, datasets, models, BT trees |
| conda env `cyclo_lerobot` | the policy backend: LeRobot 0.6 (Cyclo's fork) and its Zenoh SDK |
| `~/.cache/zenoh_ros2_sdk` | ROS message definitions for that SDK |
| `cyclo_intelligence/orchestrator/ui/build` | the web UI |

`nxp_omniman_ws/cyclo/` is outside git.

**7. Link `/workspace`** — Cyclo reads and writes its data in `/workspace`:
```bash
sudo ln -sfn ~/workspaces/nxp_omniman_ws/cyclo/workspace /workspace
```

> **Don't `conda activate` in a terminal that runs ROS.** An active env puts its `python3` ahead of
> the system one, and ROS programs that start with `#!/usr/bin/env python3` (like `rosbridge`) then
> die at once (`No module named 'tornado'`); the UI shows a ROS connection error. Run `conda
> deactivate` there, and use conda only in a separate terminal, for training.

---

## 1. Bring up the robots

Start the follower first, then the leader, then connect them.

**Terminal 1 — Zenoh router** (before any other ROS node; connects to the robot's router, leave
out `ZENOH_CONFIG_OVERRIDE` without the robot):
```bash
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.51.151:7447"]' \
    ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Terminal 2 — follower (omniman) + camera:**
```bash
ros2 launch omniman_vla nxp_omniman_launch.py
```

**Terminal 3 — leader (gravity compensation):**
```bash
ros2 launch leader_ros2_control leader_gravity_launch.py
```

Move the leader by hand so its pose roughly matches the follower **before** connecting them.

**Terminal 4 — teleop relay (the "go" button):**
```bash
ros2 launch leader_ros2_control teleop_bridges_launch.py
```

The follower now mirrors the leader, gripper included. Nothing moves until you connect the two
arms here.

**Terminal 5 — Cyclo:**
```bash
ros2 launch orchestrator omniman_cyclo_bringup.launch.py
```

Starts Cyclo's supervisor and web UI (**http://localhost:7080**), the orchestrator (with
`rosbridge` :7090, `web_video_server` :7085 and the bag recorder), `cyclo_data`, the
behaviour-tree engine, and `arm_state_relay`. The relay republishes the 7 arm joints from
`/joint_states` (which also has the wheels) on `/omniman/arm_state`, in the trained order —
Cyclo takes a state topic whole, so it needs a topic with only those joints. Topics and joints are
set in `cyclo_intelligence/shared/shared/robot_configs/omniman_config.yaml`.

Ctrl+C in terminal 5 stops everything Cyclo started. **Stop it with Ctrl+C, not by closing the
terminal:** a closed terminal leaves the orchestrator, `cyclo_data` and the tree engine running.

There is no separate UI step: the UI was built by the installer and is served by the launch.

---

## 2. Record demonstrations

In the UI: **Record** page. The robot type (`omniman`) shows at the top left.

| Field | Value |
|---|---|
| Task Num | a label for this session, e.g. `1`, `2`, … |
| Task Name | e.g. `cyclo_pick_place` |
| Task Instruction | e.g. `pick the yellow object and place it on black box` |
| Number of SubTasks | `0` — the whole recording is one episode |

The folder is named from the first two: `Task_<num>_<name>_MCAP`. Use a new Task Num for each
session so episodes don't mix.

Check the **Topic Monitor** before recording: `/omniman/arm_state` and `/leader/joint_trajectory`
at about 100 Hz, `/image_raw/compressed` at 30 Hz, all green.

Press **Record Start** (or `Space`) to start an episode and press it again to save it. There is no
warm-up, episode or reset timer: each episode lasts as long as you record, and the time between
episodes is yours. **Discard Episode** throws the current one away. **EP** at the top right counts
the episodes.

### Where the data lands

Cyclo records rosbags, not a dataset:
```
/workspace/rosbag2/Task_<num>_<name>_MCAP/
├── README.md
└── <episode>/                   # 0, 1, 2, …
    ├── <episode>_0.mcap         # /omniman/arm_state, /leader/joint_trajectory, /tf
    ├── metadata.yaml            # rosbag summary
    ├── episode_info.json        # task, robot type, length
    ├── camera_info/
    └── videos/<episode>_0/cam.mp4
```

Review or delete episodes in **Data Tools → Review Episodes / Delete Episodes**.

### Convert to a LeRobot dataset

Training needs a LeRobot dataset, so convert the recording: **Data Tools → Convert Dataset**.

| Field | Value |
|---|---|
| Task folder | `Task_<num>_<name>_MCAP` |
| FPS | **30** — the form defaults to 15, which would make a dataset at the wrong speed |
| v2.1 / v3.0 | v3.0 for Cyclo's training (LeRobot 0.6); v2.1 for physical_ai's (LeRobot 0.2) |

The result goes to `/workspace/lerobot/<task folder>_lerobot_v30` (and `_v21`). The converter
refuses to run again if that folder exists — delete or rename it first.

---

## 3. Run inference

**Shut down the leader first.**

```
Terminal 3 (leader_gravity_launch.py)    → Ctrl+C   ← STOP
Terminal 4 (teleop_bridges_launch.py)    → LEAVE RUNNING
Terminal 2 (follower + camera)           → LEAVE RUNNING
Terminal 5 (Cyclo)                       → LEAVE RUNNING
```

Cyclo publishes the policy's actions on `/leader/joint_trajectory`, the same topic the leader
uses; the teleop relay passes them to the arm. Confirm exactly one publisher:
```bash
ros2 topic info /leader/joint_trajectory
```

Then on the **Inference** page:

1. **Start the policy backend** — press **Start** on the **LeRobot** card and wait until it shows
   up. The launch does not start it.
2. **Policy Path** — the checkpoint's `pretrained_model` folder, under `/workspace/model`
   (e.g. `/workspace/model/lerobot/<run>/checkpoints/<step>/pretrained_model`).
3. **Task Instruction** — the same text as in the recording.
4. **Dataset FPS** — the dataset's FPS, **30**. A wrong value plays every motion too slow or too
   fast.
5. **Action Request** — Async or Sync; both work.
6. **Deploy Target** — **3D Sim Deploy** for a first try (the actions are not sent to the arm),
   then **Real Robot Deploy**.
7. Press **Start**.

> **First autonomous run:** hand on the e-stop, workspace clear, nothing fragile in reach.

If the policy keeps moving after the UI or the launch is gone, stop it with
```bash
src/cyclo_intelligence/native/policy/stop_policy.sh
```

### Choosing the engine

`cyclo_intelligence/native/policy/lerobot_backend.yaml` decides how an ACT policy runs; edit it,
then press **Restart** on the LeRobot card.

| `engine:` | |
|---|---|
| `lerobot_engine` | Cyclo's own (default): plays the whole action chunk from a queue. The model's temporal ensemble has no effect. |
| `omniman_act` | ACT as physical_ai ran it: one prediction per step, with the model's temporal ensemble — smoother on omniman. |

`omniman_act` uses the ensemble written in the checkpoint's `pretrained_model/config.json`:
`"temporal_ensemble_coeff": 0.01` and `"n_action_steps": 1`. Training leaves them at `null` and
`100`, so set them in the checkpoint you run.

### Models trained with physical_ai_tools

A physical_ai checkpoint (LeRobot 0.2) must be converted once before Cyclo (LeRobot 0.6) loads it.
Copy it under `/workspace/model` first, then:
```bash
~/miniconda3/envs/cyclo_lerobot/bin/python -m lerobot.processor.migrate_policy_normalization \
    --pretrained-path /workspace/model/lerobot/<model>/checkpoints/<step>/pretrained_model
```
Select the new `pretrained_model_migrated` folder as the Policy Path. Checkpoints trained with
Cyclo's LeRobot (below) need no conversion.

---

## 4. Training

Training runs from the command line, in the `cyclo_lerobot` env — the UI's **Training Guide** page
only shows a command template. It reads a LeRobot **v3.0** dataset, so use the `_lerobot_v30`
output of the conversion.

**Once:** the installer does not add LeRobot's training package, so install it into the env:
```bash
~/miniconda3/envs/cyclo_lerobot/bin/pip install 'accelerate>=1.14.0,<2.0.0'
```

**A dataset recorded with physical_ai** (v2.1) must be converted to v3.0 first. The converter
works in place, so convert a copy and keep the v2.1 original for physical_ai:
```bash
cp -r ~/dataset/<user>/<name> ~/dataset/<user>/<name>_v30
~/miniconda3/envs/cyclo_lerobot/bin/python -m lerobot.scripts.convert_dataset_v21_to_v30 \
    --repo-id=<user>/<name>_v30 --root=$HOME/dataset/<user>/<name>_v30 --push-to-hub=false
```

### The training command

In a terminal **without** ROS sourced or with `unset PYTHONPATH` (ROS's Python packages must not
mix in), with the env active:
```bash
conda activate cyclo_lerobot
unset PYTHONPATH

cat > ~/train_cmd.sh << 'EOF'
lerobot-train \
    --policy.type=act \
    --policy.device=cuda \
    --policy.push_to_hub=false \
    --dataset.repo_id=<user>/<dataset_name> \
    --dataset.root=/workspace/lerobot/<dataset_folder> \
    --dataset.image_transforms.enable=true \
    --batch_size=16 \
    --num_workers=12 \
    --steps=100000 \
    --save_freq=10000 \
    --tolerance_s=0.04 \
    --output_dir=/workspace/model/lerobot/<run_name> \
    --job_name=<run_name> \
    --wandb.enable=false
EOF
bash ~/train_cmd.sh
```

- `lerobot-train` is LeRobot 0.6's name for the old `python -m lerobot.scripts.train`.
- `--output_dir` must not exist yet; use a new `<run_name>` for each run. Under
  `/workspace/model/lerobot/` the checkpoints show up in the Inference page's Policy Path.
- `--tolerance_s=0.04` is what Cyclo's own training code passes.
- `--wandb.enable=true` also needs `pip install wandb` in the env.
- `--policy.use_amp=true` (mixed precision) is optional; it is off by default.

Checkpoints land in `<output_dir>/checkpoints/<step>/pretrained_model`. Before running one with
`omniman_act`, set the temporal ensemble in its `config.json` (see
[Choosing the engine](#choosing-the-engine)), or add
`--policy.temporal_ensemble_coeff=0.01 --policy.n_action_steps=1` to the command.

---

## If it does not work

| Message or symptom | Cause |
|---|---|
| `not installed yet - run install.sh` | the installer did not finish; run it again |
| `Directory does not exist: /workspace/model` | the `/workspace` link (install step 7) is missing |
| no topics from the robot | the Zenoh router is not running, or `ROS_DOMAIN_ID` differs |
| UI: ROS connection failed | a conda env is active in the terminal that ran the launch |
| policy waits at "starting", arm does not move | the LeRobot backend is not started (Inference page, LeRobot card) |
| `'accelerate' is required but not installed` | the training package step above |
| `Device 'cuda' is not available` in training | `CUDA_VISIBLE_DEVICES` points to a GPU that does not exist |
| `colcon build` fails with old paths | delete that package's folders in `build/` and `install/`, build again |
