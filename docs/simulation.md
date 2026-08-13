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
