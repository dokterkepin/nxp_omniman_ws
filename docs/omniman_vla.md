# omniman_vla

`omniman_vla` runs the robot for imitation-learning work: it brings up the
hardware, records data, and runs trained policies. It also holds the **control
lock** — the rule that decides whether Nav2 or a policy is allowed to move the
robot at any moment.

Nav2 itself lives in `omniman_navigation`. This package only *uses* it.

## How to run

start things in this order. Each line is its own terminal.

| # | Where | Command | What it gives you |
|---|---|---|---|
| 1 | robot PC | `ros2 launch omniman_vla nxp_omniman_launch.py` | hardware, cameras, lidar, joystick, **control lock** |
| 2 | main PC | `ros2 launch physical_ai_server physical_ai_server_bringup.launch.py` | policy inference (robot type is set to `omniman` automatically) |
| 3 | main PC | `ros2 launch omniman_navigation nav2_launch.py` | navigation |
| 4 | main PC | `ros2 launch omniman_navigation web_ui_launch.py` | iPad page on `http://<main-pc-ip>:8081` |
| 5 | main PC | `ros2 run omniman_vla pick_place_mission.py` | the autonomous mission — only when you want it |

### The problem it solves

Nav2 and a policy both drive the base over `/cmd_vel`. If both run at the same
time the base jitters, and a policy started while the robot is still driving
misses its target. Something has to decide **who is allowed to move the robot
right now**.

### The idea

The robot is a shared resource with **one owner at a time**, like a key that
only one program can hold:

1. **Ask** — a program asks for control (`acquire`). If nobody has it, it gets
   it. If someone else has it, the answer is "no, wait".
2. **Work** — it does its job, and keeps an eye out in case control is taken
   away.
3. **Hand over** — when done, it gives control back (`release`). That is the
   "I'm done" signal the next program is waiting for.

This pattern is called a **resource arbiter**, or a **mutex** (mutual
exclusion lock). In robotics it is usually called **control arbitration**.

### Who is who

```
                  omniman_interfaces   (the shared message and service types)
                          │
                   control_arbiter     the lock: knows WHO owns the robot
          publishes /control/owner, serves /control/acquire and /control/release
                ▲                              ▲
       acquire / release              acquire / release
                │                              │
     pick_place_mission              policy_runner
       (owner "nav")                (owner "policy")
                                               ▲
                                    /policy_runner/run, /stop
                                               │
                                     web UI Policy switch
```

- **control_arbiter** knows nothing about Nav2, arms or tasks. Owners are just
  names like `"nav"` or `"policy"`. That is what makes it reusable.
- **policy_runner** knows how to run a policy and when the arm is finished.
- **Missions and the web UI** decide what should happen next.

### Topics and services

| Name | Type | Purpose |
|---|---|---|
| `/control/owner` | `omniman_interfaces/msg/ControlOwner` | who owns the robot now. Latched: a program that starts later still gets the current value at once |
| `/control/acquire` | `omniman_interfaces/srv/AcquireControl` | ask for control |
| `/control/release` | `omniman_interfaces/srv/ReleaseControl` | give control back |

`ControlOwner` has `owner` (empty when nobody has control), `node` (which
node holds it) and `since` (when it was acquired).

### The rules

- **Polite by default.** `acquire` with `force: false` is refused while someone
  else holds control. Scripts always ask politely and wait their turn.
- **Force is for the operator.** `force: true` takes control from whoever holds
  it. Only the web UI uses it — that is you, overriding a script. The previous
  owner sees `/control/owner` change and must stop.
- **Only the owner can release**, unless `force: true` is used.
- **Crash safety.** When you acquire, you give your node name. If that node
  disappears (crashed, killed), the arbiter releases control after
  `holder_lost_s` (default 5 s) **plus** the time the network takes to notice
  the node is gone. With CycloneDDS that is about **15 s** in total.
- **Opt-in.** Only programs that use the lock are coordinated by it. A program
  that never calls `acquire` — RViz's 2D Goal Pose, the joystick — can still
  move the robot. The lock is an agreement between the programs that use it,
  not a wall.

### Checking it by hand

```bash
ros2 topic echo /control/owner                # who has control?

ros2 service call /control/acquire omniman_interfaces/srv/AcquireControl \
  "{owner: 'nav', node: '', force: false}"

ros2 service call /control/release omniman_interfaces/srv/ReleaseControl \
  "{owner: 'nav', force: false}"

ros2 service call /control/release omniman_interfaces/srv/ReleaseControl \
  "{owner: '', force: true}"                   # unstick: free control, whoever has it
```

---

## policy_runner

Runs one policy on physical_ai_server while holding control as `"policy"`.
It is the **only** program that starts or stops policies, which prevents two
programs from starting a policy at the same moment.

### Interface

| Name | Type | What it does |
|---|---|---|
| `/policy_runner/run` | `omniman_interfaces/srv/RunPolicy` | acquire control, start the policy |
| `/policy_runner/stop` | `std_srvs/srv/Trigger` | stop the policy, release control |
| `/policy_runner/status` | `std_msgs/msg/String` (latched) | `idle`, `starting`, `working` or `ending` |

`RunPolicy` has:
- `policy_path` — checkpoint to run. Empty = the default in `policy_runner.yaml`.
- `instruction` — task text. Empty = the default in `policy_runner.yaml`.
- `force` — passed on to the lock.

`run` returns as soon as the policy has **started**. To know when it has
**finished**, watch `/policy_runner/status` go back to `idle`.

### How it knows the arm is finished

There is no "task done" message from physical_ai_server, so the runner watches
the arm.

Every episode starts and ends with the arm at its **home pose**. The arm also
passes through home briefly as a **reset** when a grasp fails and it tries
again. So:

> **Finished = the arm left home at least once, came back, and stayed home for
> `finished_dwell_s`.**

- A reset is short, so it does not count.
- The home pose at the very start does not count, because the arm has not left
  yet.
- Only the six arm joints are checked. The gripper is ignored, because
  different tasks end with it open or closed.
- Only joint **positions** are used. At rest the reported velocities are noisy
  (spikes up to 0.5 rad/s) and would never read as still.
- Two tolerances stop flickering at the edge: the arm counts as **home** inside
  `home_tolerance` (0.20 rad), and only counts as **gone** beyond
  `home_exit_tolerance` (0.30 rad).

The log shows every step, which is how you tune it:

```
inference running - watching for the arm to finish
   left home after 0.3s (task started)
   back home
   left home after 2.4s (reset)
   back home
arm finished - home for 5.0s - ending run
```

### Other ways a run ends

It stops the policy (and releases control if it still has it) when:
- control is taken away by a forced acquire,
- inference is stopped from outside, e.g. in the physical_ai_manager UI,
- inference has not started within `warmup_timeout_s` (60 s),
- `/policy_runner/stop` is called,
- the runner shuts down.

Recordings are never touched: the runner only stops inference.

> **Watch out:** if the policy sees nothing to do and the arm **never leaves
> home**, the run never finishes, because "finished" needs the arm to leave home
> first. Stop it with the web UI's Policy switch or `/policy_runner/stop`.

### Running a policy by hand

```bash
ros2 service call /policy_runner/run omniman_interfaces/srv/RunPolicy \
  "{policy_path: '', instruction: 'pick the object', force: false}"

ros2 topic echo /policy_runner/status

ros2 service call /policy_runner/stop std_srvs/srv/Trigger
```

---

## The web UI

The iPad page is in `omniman_navigation`, but three of its controls belong to
this package:

- **Policy: ON / OFF** — reads `/control/owner`.
  - **ON** cancels any navigation and runs the default policy with
    `force: true`, so it takes over even from a running mission.
  - **OFF** stops the policy and releases control.
  - While the policy owns the robot, **Goal** mode and the place buttons are
    locked.
- **Go to** bar — one button per place in `poses.yaml`. **+ Save here** stores
  where the robot is right now.
- **Joystick: ON / OFF** — flips `teleop_enabled` on `joy_discrete_base`.

A button showing **`?`** means the page gets no answer from that node, usually
because it is not running.

---

## Writing your own commander

Any new program — in this package or any other — can use the lock. There is
nothing to register.

### 1. Depend on the types

```xml
<exec_depend>omniman_interfaces</exec_depend>
```

### 2. Pick an owner name

A short name for what is driving: `"nav"`, `"teleop"`, `"inspection"`. Two
programs using the same name count as the same owner.

### 3. Follow the pattern

```python
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

class MyCommander(Node):
    def __init__(self):
        super().__init__('my_commander')
        self.owner = ''
        self.create_subscription(ControlOwner, '/control/owner',
                                 lambda m: setattr(self, 'owner', m.owner), LATCHED)
        self.acquire_cli = self.create_client(AcquireControl, '/control/acquire')
        self.release_cli = self.create_client(ReleaseControl, '/control/release')

    def acquire(self):
        req = AcquireControl.Request()
        req.owner = 'inspection'
        req.node = self.get_fully_qualified_name()   # lets the arbiter clean up if you crash
        req.force = False                            # scripts never force
        return self.acquire_cli.call_async(req)      # result.success False = wait and retry

    def release(self):
        req = ReleaseControl.Request()
        req.owner = 'inspection'
        return self.release_cli.call_async(req)

    def still_mine(self):
        return self.owner == 'inspection'
```

### 4. The rules

- **Refused?** Someone else has control. Wait about half a second and ask again.
- **While working, check `still_mine()`.** If it turns false, you were
  overridden: stop at once. Do not release — it is not yours any more.
- **Release on every exit** — success, failure and Ctrl+C. Use `finally`.
- **Always fill in `node`**, so a crash cannot leave the robot locked.
- **Never use `force: true` in a script.** It is for the operator.

### 5. Running a policy from your commander

Do not call physical_ai_server directly. Hand over to policy_runner:

```
release your owner → call /policy_runner/run → wait for status "idle" → acquire your owner again
```

The `Control` and `Policy` classes in `commander/pick_place_mission.py` already
do all of this and follow the rules above — copy them to start.

---

## Config files

| File | Used by | What is in it |
|---|---|---|
| `mission.yaml` | both missions | policy paths and instructions (`manipulate`, `base_correction`), `fps`, `still_time_s`, `settle_timeout_s`. The `*_duration_s` values are only used by `pick_place_shuttle.py` |
| `poses.yaml` | missions, web UI | named places in the map frame, yaw in **degrees**. Written by the web UI's **+ Save here**, so comments you add are not kept |
| `policy_runner.yaml` | policy_runner | default policy and instruction, `home_pose`, `home_tolerance`, `home_exit_tolerance`, `finished_dwell_s`, `warmup_timeout_s` |
| `joystick_discrete.yaml` | joy_discrete_base | button and axis numbers, speed levels, `teleop_enabled` |
| `controllers_vla.yaml` | nxp_omniman_launch | ros2_control controllers |

### Tuning the "finished" check

| Parameter | Default | Change it when |
|---|---|---|
| `home_pose` | measured at rest | you change the arm's home position. Capture it again from `/joint_states` |
| `home_tolerance` | 0.20 rad | the arm comes to rest a bit off home and is never counted as home |
| `home_exit_tolerance` | 0.30 rad | normal jitter near home is logged as "left home" |
| `finished_dwell_s` | 5.0 s | a reset is mistaken for a finish — set it longer than your longest `(reset)` pause in the log |
