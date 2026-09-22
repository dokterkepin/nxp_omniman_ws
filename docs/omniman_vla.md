# omniman_vla

`omniman_vla` turns the robot into something a **mission** can drive: a
mobile base (Nav2), a learned arm policy (ACT / SmolVLA), a camera that finds
targets (SAM 3), and the glue that lets them take turns safely.

This page has four parts:

0. **[Setup](#0-setup)** - what to install, once per PC.
1. **[The big picture](#1-the-big-picture)** - what runs where, and how to start it.
2. **[The building blocks](#2-the-building-blocks)** - one section per node, with
   its **contract**: what you send it, what it answers, and the rules for using
   it.
3. **[Tutorial: write a mission](#3-tutorial-write-a-mission)** - how
   `pick_place_bt.py` is built, step by step, so you can build your own.

Nav2 itself lives in `omniman_navigation`; this package only *uses* it.

---

## 0. Setup

Do this once per PC. The **robot PC** only needs the ROS workspace (step 1).
The **GPU PC** runs the detectors, which need a conda env (steps 2-4).

### 1. ROS packages (every PC)

```bash
sudo apt install ros-jazzy-vision-msgs ros-jazzy-py-trees ros-jazzy-py-trees-ros
cd ~/workspaces/nxp_omniman_ws && colcon build --symlink-install
```

`vision_msgs` carries the detections, `py_trees` runs the behaviour-tree
missions (`pick_place_bt.py`).

### 2. The conda env (GPU PC)

All Python packages are pinned in `omniman_vla/requirements.txt`. One env holds
both detectors (SAM 3 and EfficientSAM3) and runs everything
`control_launch.py` starts.

Open a **new terminal without ROS sourced** - with ROS sourced, pip also sees
ROS's own Python packages and gets confused:

```bash
conda create -n omniman_vla python=3.12 -y
conda activate omniman_vla
pip install -r ~/workspaces/nxp_omniman_ws/src/omniman_vla/requirements.txt
```

Then EfficientSAM3's model code. It must be an **editable** install from a
clone - its packaging leaves out part of the code, so a normal
`pip install` builds a model that cannot load:

```bash
git clone https://github.com/SimonZeng7108/efficientsam3 ~/tools/efficientsam3
git -C ~/tools/efficientsam3 checkout bd0936c788fed8d51fa799437f05abd97b401b06
pip install -e ~/tools/efficientsam3/sam3
```

Check:

```bash
pip check                                   # "No broken requirements found."
python -c "import torch; print(torch.cuda.is_available())"   # True
```

> The PyTorch wheels in `requirements.txt` are built for **CUDA 13**, which
> needs a recent NVIDIA driver (`nvidia-smi` shows "CUDA Version: 13.x" or
> newer). With an older driver, install PyTorch from the matching index first,
> e.g. `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126`,
> then the rest of the file.

### 3. Model weights (GPU PC)

Everything goes in `~/models`.

**SAM 3** - Meta's repo is gated: log in at
[huggingface.co/facebook/sam3](https://huggingface.co/facebook/sam3), request
access (usually approved within a day), then:

```bash
hf auth login                                   # once, with a Hugging Face token
hf download facebook/sam3 sam3.pt --local-dir ~/models
```

**EfficientSAM3** - open, no request:

```bash
hf download Simon7108528/EfficientSAM3 efficientsam3_ft/efficientsam3_repvit.pt \
    --local-dir ~/models/efficientsam3
```

(`efficientsam3_efficientvit.pt` and `efficientsam3_tinyvit.pt` are the other
two variants, same command.)

SAM 3 also downloads its CLIP text model (~350 MB) the first time it starts.
Keep it in `~/models` rather than in whatever folder you started from:

```bash
yolo settings weights_dir=$HOME/models
```

### 4. Using the env

Every terminal that starts `control_launch.py` or a detector:

```bash
conda activate omniman_vla
source /opt/ros/jazzy/setup.bash
source ~/workspaces/nxp_omniman_ws/install/setup.bash
```

Order matters: conda first, then ROS - the nodes then run on the env's Python
and find ROS's packages through `PYTHONPATH`.

---

## 1. The big picture

### What a mission does

A mission is a list of steps, each handled by one building block:

```
drive to the table        -> Nav2                     (owner "nav")
line the base up          -> visual_align + detector  (owner "align")
pick the cup              -> policy_runner            (owner "policy")
did it really pick?       -> grasp_monitor            (no owner, only reads)
drive to the place        -> Nav2                     (owner "nav")
line up with the mark     -> visual_align + detector
place the cup             -> policy_runner
drive home                -> Nav2
```

Only **one** step may move the robot at a time. That is the **control lock**
(`control_arbiter`): whoever moves the robot must hold it, under an **owner
name**. The mission never moves motors itself - it asks the building blocks,
and each of them takes and gives back the lock on its own.

```
                               pick_place_bt.py  (the mission)
          ┌────────────────┬────────────┴───────┬──────────────────┐
     Nav2 goals      /visual_align/run    /policy_runner/run    /gripper/holding
    (owner "nav")           │                   │                (reads only)
          │            visual_align        policy_runner          grasp_monitor
          │           (owner "align")    (owner "policy")              │
          │                 │                   │                 /joint_states
          │     /sam_detector/detections  physical_ai_server
          │                 │              (ACT / SmolVLA)
          │         sam_detector (SAM 3)
          │                 │
          └──── all take and give back the lock at control_arbiter ────┘
```

### What runs where

| # | PC | Command | Gives you |
|---|---|---|---|
| 1 | robot PC | `ros2 launch omniman_vla nxp_omniman_launch.py` | motors, wrist camera, lidar, joystick |
| 2 | GPU PC | `ros2 launch physical_ai_server physical_ai_server_bringup.launch.py` | runs the arm policy |
| 3 | GPU PC, in the `omniman_vla` env ([Setup](#4-using-the-env)) | `ros2 launch omniman_vla control_launch.py` | control_arbiter, policy_runner, detector, visual_align, grasp_monitor |
| 4 | robot or GPU PC | `ros2 launch omniman_navigation nav2_launch.py` | Nav2 **and** the web UI (`http://<that-pc>:8081`) |
| 5 | same PC as Nav2 | `ros2 run omniman_vla pick_place_bt.py` | the mission |

- **Run the mission on the same PC as Nav2.** The web UI's "Save here" writes
  `poses.yaml` on the Nav2 PC, and the mission reads `poses.yaml` on its own PC.
- **Every node reads the config files of the PC it runs on.** Edit them there
  (or commit and `git pull`).
- **Only one copy of each launch on the network.** Two detectors or two
  `visual_align`s publish over each other.

---

## 2. The building blocks

Each block below has the same layout: **what it is for**, its **interface**
(topics and services), its **contract** (the rules you must follow when you use
it), and how to **try it by hand**.

### 2.1 control_arbiter - the lock

**What for.** Nav2, visual_align and the policy all move the base over
`/cmd_vel`. If two run at once the base jitters, and a policy started while the
base is still moving misses. The arbiter decides **who may move the robot right
now**: one owner at a time, like a key only one program can hold. This pattern
is called a **resource arbiter** or **mutex**.

**Interface**

| Name | Type | Purpose |
|---|---|---|
| `/control/owner` | `omniman_interfaces/msg/ControlOwner` (latched) | who owns the robot: `owner` (empty = nobody), `node`, `since` |
| `/control/acquire` | `omniman_interfaces/srv/AcquireControl` | ask for control: `owner`, `node`, `force` |
| `/control/release` | `omniman_interfaces/srv/ReleaseControl` | give it back: `owner`, `force` |

*Latched* means a program that starts later still gets the current value at
once.

**Contract**

1. **Ask politely.** `acquire` with `force: false`. If refused, someone else
   has it - wait about 0.5 s and ask again.
2. **Always fill in `node`** (`self.get_fully_qualified_name()`). If your node
   crashes, the arbiter frees the lock after `holder_lost_s` (5 s) plus DDS
   discovery time - about 15 s in total.
3. **While working, watch `/control/owner`.** If it stops showing your owner
   name, you were overridden: **stop at once, and do not release** - it is not
   yours any more.
4. **Release on every exit:** success, failure and Ctrl+C. Use `finally`.
5. **Scripts never use `force: true`.** Force is for the operator (the web UI).
6. **It is opt-in.** Programs that never acquire (RViz's 2D Goal Pose, the
   joystick) can still move the robot. The lock is an agreement, not a wall.

**Owner names in use:** `nav` (missions driving with Nav2), `align`
(visual_align), `policy` (policy_runner).

**By hand**

```bash
ros2 topic echo /control/owner
ros2 service call /control/acquire omniman_interfaces/srv/AcquireControl "{owner: 'nav', node: '', force: false}"
ros2 service call /control/release omniman_interfaces/srv/ReleaseControl "{owner: 'nav', force: false}"
ros2 service call /control/release omniman_interfaces/srv/ReleaseControl "{owner: '', force: true}"   # unstick
```

### 2.2 policy_runner - run one arm policy until it is done

**What for.** Starts a policy on physical_ai_server under the lock (owner
`policy`), and decides **when the arm is finished**, because physical_ai_server
has no "task done" message. It is the only program that starts or stops
policies.

**Interface**

| Name | Type | What it does |
|---|---|---|
| `/policy_runner/run` | `omniman_interfaces/srv/RunPolicy` | take the lock, start the policy. `policy_path` / `instruction` empty = defaults from `policy_runner.yaml`; `force` goes to the lock |
| `/policy_runner/stop` | `std_srvs/srv/Trigger` | stop the policy, release the lock |
| `/policy_runner/status` | `std_msgs/msg/String` (latched) | `idle`, `starting`, `working`, `ending` |

**How "finished" is decided.** Every episode starts and ends with the arm at
its home pose, and passes through home briefly as a reset between attempts. So:

> **Finished = the arm left home at least once, came back, and stayed home for
> `finished_dwell_s` (10 s).**

Only the six arm joint **positions** are checked (not the gripper, not the
noisy velocities). Home is inside `home_tolerance` (0.20 rad); gone is beyond
`home_exit_tolerance` (0.30 rad). The log shows each step:

```
inference running - watching for the arm to finish
   left home after 0.3s (task started)
   back home
   left home after 2.4s (reset)
   back home
arm finished - home for 10.0s - ending run
```

A run also ends when control is taken away, inference is stopped from
outside, inference has not started within `warmup_timeout_s` (60 s), or
`/policy_runner/stop` is called.

**Contract**

1. **`run` returns when the policy has *started*, not finished.** Watch
   `/policy_runner/status`: finished = back to `idle` **after having been busy**.
   The status is latched, so an old `idle` is already there before your run
   starts - ignore `idle` until you have seen `starting` or `working`.
2. **Do not hold your own owner while the policy runs.** Release it, call
   `run`, wait for idle, acquire again.
3. **Stop it on your exit** (`/policy_runner/stop`) - it is a no-op when idle.
4. **Know its blind spot:** if the arm never leaves home, or never comes back
   and settles (for example it keeps reaching for a target you moved), the run
   never finishes. Your mission needs its own time limit or a stop button.

**By hand**

```bash
ros2 service call /policy_runner/run omniman_interfaces/srv/RunPolicy "{policy_path: '', instruction: 'pick the object', force: false}"
ros2 topic echo /policy_runner/status
ros2 service call /policy_runner/stop std_srvs/srv/Trigger
```

### 2.3 The detector - find targets by text (SAM 3)

**What for.** Finds objects in the wrist camera from short text prompts
("yellow cup lid", "black square") and reports where they are in the image.
There are two interchangeable versions, switched by hand in
`control_launch.py`:

| Node | Model | Notes |
|---|---|---|
| `sam_detector` | SAM 3 (Ultralytics) | best quality; ~35 ms at `imgsz: 336`, 3.4 GB GPU |
| `efficient_sam_detector` | EfficientSAM3 (distilled) | ~65 ms, ~1.1 GB; cup fine, flat black mark weak |

Both run in the `omniman_vla` env.

**Interface**

| Name | Type | Purpose |
|---|---|---|
| `/<detector>/detections` | `vision_msgs/msg/Detection2DArray` | one detection per prompt found: `class_id` = **the prompt text**, `score`, centre = the mask's centre in pixels (640x480) |
| `/<detector>/debug/compressed` | `sensor_msgs/msg/CompressedImage` | the camera image with masks, scores and `<- align` drawn |

**Contract**

1. **The name of a detection is the prompt, word for word.** Anything that
   looks for "black square" only finds it if a prompt is exactly "black square".
2. **Detections come in prompt order**, best mask per prompt.
3. **It only works while someone listens** - it runs nothing without a
   subscriber, so it costs no GPU when idle. Seeing the debug image keeps it
   running.
4. **Prompts that work:** short noun phrases describing what the camera sees.
   Tested: "yellow cup lid", "black square". Avoid "black mark" (matched dirt
   specks). Test new prompts on the debug image before using them.

**By hand**

```bash
ros2 run rqt_image_view rqt_image_view /sam_detector/debug/compressed
ros2 topic echo /sam_detector/detections --field detections
```

### 2.4 visual_align - move the base until the target is in place

**What for.** The last centimetres Nav2 cannot do: moves the base (turn +
forward/back) until the target sits at the pixel where the policy expects it
(`aim_x`, `aim_y`). A P-controller in pixels - no depth, no markers.

**Interface**

| Name | Type | Purpose |
|---|---|---|
| `/visual_align/run` | `std_srvs/srv/Trigger` | take the lock (owner `align`, polite) and start |
| `/visual_align/stop` | `std_srvs/srv/Trigger` | stop the base, release |
| `/visual_align/status` | `std_msgs/msg/String` (latched) | `idle`, `starting`, `searching`, `aligning`, `aligned`, `failed: <why>` |
| `/visual_align/target` | `std_msgs/msg/String` (latched, **input**) | which detection to align to, by name (e.g. `"black square"`). Empty = the first detection |

**What a run does**

```
searching   target not seen yet: turn continuously in search_direction
            until search_turns full turns (from odometry), then fail
aligning    target seen: turn (x error) and drive forward/back (y error),
            each a P-controller with a minimum and maximum speed
lost        detections drop out (glare, blur): keep aligning on the last
            detection; fail after lost_s without a new one. Once seen,
            it never searches again
aligned     target inside tolerance_x / tolerance_y for settle_frames
            detections in a row -> stop, release the lock
```

**Contract**

1. **Set `/visual_align/target` before `run`** when more than one prompted
   object can be in view (at the place, the held cup is in view too). The name
   must be **exactly** a detector prompt.
2. **Watch `/visual_align/status` like policy_runner's:** the result is latched
   (`aligned` / `failed: ...` stays), so only a result after `searching` /
   `aligning` belongs to your run.
3. **After `aligned`, wait for the base to be still** (odometry) before starting
   the arm - it may still be coasting.
4. **Stop it on your exit** (`/visual_align/stop`).

**By hand**

```bash
ros2 topic pub --once /visual_align/target std_msgs/msg/String "{data: 'yellow cup lid'}"
ros2 service call /visual_align/run std_srvs/srv/Trigger
ros2 topic echo /visual_align/status
```

### 2.5 grasp_monitor - is the gripper holding something?

**What for.** The policy closes the gripper whether or not the cup is there, so
"closed" says nothing. How far it closes and how hard it pushes do:

| | Position | Effort |
|---|---|---|
| closed on the cup | stays above `closed_empty_position` (-0.002) - the cup stops the fingers | pushing: \|effort\| above `holding_min_effort` |
| closed on nothing | goes past -0.002 toward fully closed (-0.013) | near 0 |
| open | ~0.019 | near 0 |

This is the standard grasp check (ROS 2's parallel_gripper_action_controller
"stalled", QT-Opt's "gripper not fully closed"), done on `/joint_states`
because the policy drives the gripper through `arm_controller`.

**Interface**

| Name | Type | Purpose |
|---|---|---|
| `/gripper/holding` | `std_msgs/msg/Bool` (latched) | true = holding |
| `/grasp_monitor/state` | `std_msgs/msg/String` (latched) | `holding` / `empty - closed on nothing` / `empty - open`, with the position and effort |

**Contract**

1. **Read it after the gripper has settled** (it waits `stable_s`, 0.3 s,
   before changing). After a policy run has finished, it has.
2. **It only reads** - no lock, no owner.
3. **Tune the two thresholds** in `mission.yaml` (`grasp_monitor:`) from real
   readings: `ros2 topic echo /grasp_monitor/state` with the gripper on the
   object, closed on nothing, and open.

### 2.6 The web UI switches

The page lives in `omniman_navigation` and starts with Nav2. Two switches use
this package:

- **Align: ON/OFF** - cancels navigation, then `/visual_align/run` (polite:
  refused while someone else holds the lock). Turns itself off at `aligned` /
  `failed: ...` and shows the result.
- **Policy: ON/OFF** - cancels navigation, then `/policy_runner/run` with
  `force: true` (the operator overrides anything). OFF stops it.

While either owns the robot, Goal mode and the place buttons are locked. A
switch showing `?` means that node is not answering - `control_launch.py` is
not running.

---

## 3. Tutorial: write a mission

A mission is a **behaviour tree**: the steps of the task arranged so that
retries, checks and "stop if X" are part of the structure instead of `if` and
`while` spaghetti. The steps are ready-made in the library
`omniman_vla.mission` - a new mission is mostly drawing its tree. This part
shows how, with `commander/pick_place_bt.py` as the worked example.

We use **py_trees** (Python; the same ideas as BehaviorTree.CPP, which Nav2
uses). Install once on the PC that runs the mission:

```bash
sudo apt install ros-jazzy-py-trees ros-jazzy-py-trees-ros
```

### Step 0 - behaviour trees in five minutes

The tree is **ticked** ten times a second (`tick_s`). Each tick, every active node
returns one of:

| Status | Meaning |
|---|---|
| `RUNNING` | still working - tick me again |
| `SUCCESS` | done, it worked |
| `FAILURE` | done, it did not |

The building bricks we use:

| Brick | Does |
|---|---|
| `Sequence(memory=True)` | runs its children in order; stops at the first `FAILURE`; `SUCCESS` when all succeed. `memory=True` = a finished child is not re-run on the next tick |
| `Retry(n)` | re-runs its child after a `FAILURE`, up to `n` failures in total |
| `EternalGuard(condition)` | re-checks `condition()` **every tick** while its child runs; the moment it is false, the child is stopped and the guard fails |
| a **step** (your class) | starts something, follows it, reports the result |
| a **condition** (your class) | checks something once: `SUCCESS` or `FAILURE` |

A step is a class with three methods py_trees calls for you:

```
initialise()          once, when the step starts        -> start the thing
update()  -> Status   every tick while it runs          -> follow it
terminate(new_status) once, when it ends or is stopped   -> clean up / cancel
```

**Golden rule: never block.** `update()` must return at once. Start a service
call in one tick, check `future.done()` in the next ones.

### Step 1 - write the mission down as a tree

Before code, draw it. For pick and place:

```
pick and place                       Sequence
 ├─ nav to pick_area                 Navigate
 ├─ pick attempts                    Retry(max_pick_attempts)
 │    └─ attempt                     Sequence
 │         ├─ align to pick_target   Align
 │         ├─ pick policy            PolicyStep
 │         └─ grasp succeeded        Holding(True)
 ├─ while holding                    EternalGuard(holding)
 │    └─ nav to place_area           Navigate
 ├─ place attempts                   Retry(max_place_attempts)
 │    └─ attempt                     Sequence
 │         ├─ align to place_target  Align
 │         ├─ place policy           PolicyStep
 │         └─ cup released           Holding(False)
 └─ nav to home                      Navigate
```

Read it aloud: *drive there; try up to 3 times to align, pick and confirm the
grasp; drive to the place but abort if the cup drops; try up to 3 times to
align, place and confirm the release; drive home.* Each question "what if X
fails?" gets its answer from the structure: a `Retry` around it, a guard, or
letting the whole mission fail.

### Step 2 - build it from the library

You do not write the steps yourself. They are in the library
**`omniman_vla.mission`**; a mission only draws its tree and calls
`run_mission`. Here is a complete mission for another task - fetch a bottle
and bring it to a person:

```python
#!/usr/bin/env python3
"""Fetch a bottle from the shelf and bring it to the person."""
import py_trees
from omniman_vla.mission import Align, Holding, Navigate, PolicyStep, run_mission

BOTTLE_POLICY = '~/output/omniman_fetch_bottle/checkpoints/last/pretrained_model'


def build(robot):
    pick = py_trees.decorators.Retry('pick attempts', py_trees.composites.Sequence(
        'attempt', memory=True, children=[
            Align(robot, 'green bottle'),                           # a detector prompt
            PolicyStep(robot, 'pick', 'pick the bottle', policy_path=BOTTLE_POLICY),
            Holding(robot, 'grasp succeeded', holding=True),
        ]), num_failures=3)

    return py_trees.composites.Sequence('fetch bottle', memory=True, children=[
        Navigate(robot, 'shelf'),                                   # a place in poses.yaml
        pick,
        py_trees.decorators.EternalGuard(                           # abort if it drops
            'while holding', Navigate(robot, 'person'), condition=lambda: robot.holding),
    ])


if __name__ == '__main__':
    run_mission(build, node_name='fetch_bottle')
```

That is the whole file. `commander/pick_place_bt.py` is built the same way -
read it next to this page.

### Step 3 - what the library gives you

**The steps**

| Step | Does | Arguments |
|---|---|---|
| `Navigate(robot, place)` | drive with Nav2 to a place from `poses.yaml`, wait until the base is still. Holds owner `nav` only while driving | `place`: name in `poses.yaml` |
| `Align(robot, target)` | `visual_align` to a target, wait until the base is still | `target`: **exactly** a detector prompt |
| `PolicyStep(robot, label, instruction, policy_path='')` | run an arm policy through `policy_runner` until the arm is finished | `label`: name in the tree; `instruction`: the policy's task text; `policy_path`: checkpoint, empty = `policy_runner.yaml`'s default |
| `Holding(robot, name, holding=True)` | condition on `grasp_monitor` | `holding=True`: SUCCESS if holding (a pick worked); `False`: SUCCESS if not (a place let go) |

Each one already follows its building block's contract from part 2: waits
until the service exists, reads the latched status correctly, cancels what it
started if the tree interrupts it, and writes its reason next to itself in the
printed tree:

```
--> align to "yellow cup lid" [✓] -- aligned in 7.4s, base turned +45 deg, moved 0.16 m
--> grasp succeeded [✕] -- empty - closed on nothing (position -0.0120, effort -1)
```

**`run_mission(build, node_name, initial_pose='home')`** does everything
around the tree:

- ROS init with a Ctrl+C that still lets it clean up
- reads `mission.yaml` (the `mission_file` parameter) and `poses.yaml` beside it
- **checks every `Align` target** against the running detector's prompts, and
  stops at once if one is not a prompt - finds them in your tree by itself
- gives AMCL the start pose (`initial_pose`: a place in `poses.yaml`, or `None`)
  and waits for Nav2
- ticks the tree, prints it whenever a status changes
- on **any** exit - done, failed, Ctrl+C - stops Nav2, `visual_align` and the
  policy, and releases `nav`

**`robot`** - the object `build(robot)` receives. What you may use in your tree:

| | |
|---|---|
| `robot.cfg` | the whole `mission.yaml`, plus `robot.cfg['poses']` |
| `robot.holding` | latest `/gripper/holding` (e.g. for an `EternalGuard`) |
| `robot.pose` | `(x, y, yaw)` from odometry |

### Step 4 - settings

Everything tunable is in `config/mission.yaml`, `settings:` - read once when
the mission starts:

| Setting | Used for |
|---|---|
| `still_time_s`, `still_linear`, `still_angular` | when the base counts as stopped (after driving and aligning) |
| `settle_timeout_s` | how long to wait for that before failing |
| `service_wait_s` | how long a step waits for a service before "not answering" |
| `tick_s` | how often the tree is ticked |

Your own mission's values (attempts, targets, instructions) can go in the same
file - `robot.cfg['settings']['max_pick_attempts']` is how `pick_place_bt.py`
reads its own. Use a different file with `--ros-args -p mission_file:=...`.

### Step 5 - install and run

1. Put the file in `commander/`, `chmod +x` it.
2. Add it to `install(PROGRAMS ...)` in `CMakeLists.txt`.
3. `colcon build --packages-select omniman_vla`
4. With part 1's launches running: `ros2 run omniman_vla fetch_bottle.py`

A mission in **another package** works too: `from omniman_vla.mission import ...`
and add `<exec_depend>omniman_vla</exec_depend>` to its `package.xml`.

### Step 6 - a new kind of step

When a competition needs something the four steps do not do - open a door,
wait for a person, call a new service - write a step. Subclass
`omniman_vla.mission.Step`; it gives you `fail()`, `succeed()`, `send()`,
`interrupted()` and `settle()`. The shape is always the same:

```python
import time

from py_trees.common import Status
from std_srvs.srv import Trigger
from omniman_vla.mission import Step


class OpenDoor(Step):
    """Ask the door node to open, wait for "open"."""

    def __init__(self, robot):
        super().__init__('open door', robot)
        self.client = robot.nav.create_client(Trigger, '/door/open')

    def initialise(self):                      # the step starts
        self.future, self.since = None, time.monotonic()

    def update(self):                          # every tick - never block
        if self.future is None:
            self.future = self.send(self.client, Trigger.Request())  # waits for the service
            return Status.FAILURE if self.future == 'gone' else Status.RUNNING
        if not self.future.done():
            return Status.RUNNING
        res = self.future.result()
        return self.succeed('door open') if res.success else self.fail(res.message)

    def terminate(self, new_status):           # interrupted? undo what you started
        if self.interrupted(new_status):
            pass                               # e.g. call a stop service
```

The rules every step keeps:

1. **Never block** in `update()` - start in one tick, check `future.done()` in
   the next ones.
2. **Send only when the service exists** - use `self.send()`; a request sent
   before discovery can be lost silently.
3. **Latched results:** if the thing you follow reports its result on a
   latched topic, only trust a result after you have seen it busy.
4. **Cancel in `terminate()`** when `self.interrupted(new_status)`.
5. **Give reasons** with `fail(...)` / `succeed(...)` - they appear in the tree.
6. **If it moves the robot, it must take the lock** (part 2.1) - or better,
   make it a node with a run/stop/status interface like `visual_align`, and a
   step that calls it.

If the step is useful for more than one mission, add it to
`omniman_vla/omniman_vla/mission.py` so the next person can import it.

### Step 7 - test without the robot

Most mistakes are in the tree, and the tree can be tested on any PC: run the
real `control_arbiter` and `grasp_monitor`, and small fake nodes for the rest -
a fake `/visual_align/run` that reports `aligned` after a second, a fake
`/policy_runner/run` that goes `working` then `idle`, fake `/joint_states` for
the gripper, and a stand-in object with `goToPose` / `isTaskComplete` /
`getResult` / `cancelTask` in place of `BasicNavigator`. Build your tree with
`omniman_vla.mission.Robot(fake_nav, cfg)` and tick it yourself. Use a separate
domain so nothing reaches the robot:

```bash
export ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Script the cases that matter - works; fails once then works; always fails;
the object drops - and check the tree ends the way you expect. That is how
`pick_place_bt.py` was tested before it ran on the robot.

### Checklist for a new mission

- [ ] Drew the tree first; every failure has an answer (retry, guard, or abort).
- [ ] Built it from `omniman_vla.mission`; new steps subclass `Step` and keep
      its rules.
- [ ] Align targets are written exactly like detector prompts (checked at
      start anyway).
- [ ] Places exist in `poses.yaml` on the PC that runs the mission.
- [ ] Tested against fakes in domain 99 before the robot.

---

## Config files

| File | Read by | What is in it |
|---|---|---|
| `mission.yaml` | missions, grasp_monitor | policy path and instructions; `settings:` (`still_time_s`, `settle_timeout_s`, `align_before_pick/place`, `max_pick/place_attempts`, `pick/place_target`); `grasp_monitor:` thresholds |
| `poses.yaml` | missions, web UI | named places in the map frame, yaw in **degrees**. Written by the web UI's "Save here" - comments are not kept |
| `visual_align.yaml` | detectors, visual_align | detector sections (`prompts`, model, `conf`, ...) and `visual_align:` (aim point, tolerances, gains, search, `detections_topic`) |
| `policy_runner.yaml` | policy_runner | default policy, `home_pose`, tolerances, `finished_dwell_s`, `warmup_timeout_s` |
| `controllers_vla.yaml` | robot bringup | ros2_control controllers |

`visual_align.yaml` and the `grasp_monitor:` section are read by the nodes
**directly from the file** and reloaded within a second of saving - no
restart, no `ros2 param set`. Topic names are read only at start.

## Tuning

| What | Setting | Change it when |
|---|---|---|
| "arm finished" | `finished_dwell_s` (10 s) | a reset is mistaken for a finish: make it longer than the longest `(reset)` pause in the log |
| | `home_pose`, `home_tolerance` | the arm rests a bit off home and is never counted as home |
| alignment | `aim_x`, `aim_y` | the policy's grasp starts from a different view; move the robot where it should stop and read the target's pixel from the detections |
| | `tolerance_x/y` (30 px) | it hunts back and forth near the target: widen; stops too early: tighten |
| | `k_angular`, `k_forward`, `min_*`, `max_*` | too slow / overshoots; `min_*` below the speed that moves the base at all does nothing |
| | `search_turns`, `search_direction` | how far and which way to look when the target is not in view |
| grasp check | `closed_empty_position`, `holding_min_effort` | real readings from `/grasp_monitor/state` sit on the wrong side of a threshold |

 the gripper | `holding_min_effort` above the real effort - check `/grasp_monitor/state` and lower it |
