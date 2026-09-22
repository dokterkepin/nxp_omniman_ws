# Why `use_sim` Matters
The `use_sim:=true` parameter controls several critical differences between simulation and
real hardware:

**Clock source (wall time vs sim time)**

Real hardware uses the system **wall clock**. Isaac Sim uses its own **simulation clock** which
runs at a different rate. When `use_sim:=true`, all nodes set `use_sim_time: true`, meaning they
read time from the `/clock` topic published by Isaac Sim instead of the system clock. If you
forget this, TF timestamps (wall time) won't match sensor timestamps (sim time), and you get
errors like "no map received" or TF extrapolation warnings.

> **Important:** In Isaac Sim, the action graph **must** have `Isaac Read Simulation Time`
> connected to all ROS 2 publisher timestamp inputs AND to `ROS2 Publish Clock`. Without
> `/clock` being published, `use_sim_time: true` has no effect.

**Hardware interface**

| | Real hardware | Simulation |
|---|---|---|
| Wheels | CyberGear motors via CAN bus | `TopicBasedSystem` plugin (`/isaac_wheel_states`) |
| Arm | CyberGear + Dynamixel via CAN/serial | `TopicBasedSystem` plugin (`/isaac_arm_states`) |
| Lidar | RPLidar physical sensor | Isaac Sim lidar sensor (publishes `/scan` directly) |

When `use_sim:=true`, the URDF xacro switches from real hardware plugins to `TopicBasedSystem`,
and the SLAM launch skips launching the physical RPLidar node.

# Why the sim mirrors the real robot

If Isaac Sim is open while the **real** hardware launch is running, moving the
real arm moves the sim arm 1:1. This is not a sync of two clocks, it is just a topic subscription:

```
real motors → ros2_control (wall clock) → joint_state_broadcaster → /joint_states
                                                                        │ DDS (same ROS_DOMAIN_ID)
Isaac Sim OmniGraph: ROS2 Subscribe Joint State ────────────────────────┘
                     → Articulation Controller (position targets) → sim physics
```

- `/joint_states` carries joint **names and positions**. The OmniGraph subscriber reads the
  latest message and sets those positions as drive targets on the matching joints.
- It **ignores the message timestamp**. It only applies "the newest value I have" on each sim
  tick. So the real robot stamps with wall time, the sim runs on its own sim clock, and nothing
  compares them, which means no mismatch error.
- The sim lags the real arm by about one physics step plus network delay (a few ms), so it looks
  1:1. It is a *follower*, not a synchronized twin: if physics blocks a joint (collision, weak
  drive gains), the sim pose will differ and nothing corrects it back.
