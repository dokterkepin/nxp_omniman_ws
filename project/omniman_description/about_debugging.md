# TF & ROS2 Debugging Guide for NXP Omniman

### 1. View the full TF tree

```bash
ros2 run tf2_ros view_frames
```

Saves a PDF (`frames_<timestamp>.pdf`) showing all frames, their parent-child relationships, broadcast rates, and timestamps. Use this first to get the big picture.

### 2. Check a specific transform between two frames

```bash
ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>
```

Examples:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

Shows live translation, rotation (quaternion, RPY in radians and degrees), and the 4x4 transformation matrix. Useful for verifying a specific link in the chain.

Note: you may see a "frame does not exist" warning on the first second if the publisher hasn't sent its first message yet. This is normal and resolves itself.

### 3. Monitor all TF broadcasters and rates

```bash
ros2 run tf2_ros tf2_monitor
```

Shows all active transforms, their broadcast rates, and delay statistics. Useful for spotting slow or missing publishers.

### 4. Check the static transforms

```bash
ros2 topic echo /tf_static --once
```

Shows all static transforms (published once with latching). These come from `robot_state_publisher` based on the URDF.

### 5. Check dynamic transforms

```bash
ros2 topic echo /tf --once
```

Shows the latest dynamic transforms (odom, joint states, etc.).

---

## Topic Debugging Tools

### 1. Check if a topic is publishing and at what rate

```bash
ros2 topic hz /scan
```

Prints the publishing rate in Hz every second. If nothing appears, the topic is not being published. Example output:

```
average rate: 12.762
    min: 0.055s max: 0.092s std dev: 0.00452s window: 157
```

This tells you the lidar is publishing at ~12.76 Hz. The rate may fluctuate with a small window (few samples) and stabilize as more samples accumulate. Small variations (e.g. 12.886 → 12.762) are normal convergence, not a real drop.

### 2. See the actual data on a topic

```bash
ros2 topic echo /scan --once
```

Prints one full message and exits. For `/scan` this shows the laser ranges, angle limits, and timestamp. For `/tf` it shows the transform data. 

### 3. Check who is publishing and subscribing to a topic

```bash
ros2 topic info /cmd_vel --verbose
```

Shows all publishers and subscribers on a topic, including their node names, message types, and QoS settings. Useful for diagnosing connection problems like type mismatches (e.g. `Twist` vs `TwistStamped` on the same topic).

Key things to look for:
- **Subscription count: 0** means nobody is listening — messages go nowhere
- **Different types** on the same topic (e.g. `Twist` publisher + `TwistStamped` subscriber) means they will never connect

### 4. List all active topics

```bash
ros2 topic list
```

### 5. Check if a node is running

```bash
ros2 node list
```

---

## Time & Clock Debugging

### Check wall clock time

```bash
date +%s.%N
```

Prints seconds since epoch (Jan 1, 1970) with nanosecond precision. Run on both robot PC and laptop to compare — if they differ by more than ~0.1 seconds, NTP might not be syncing properly.

### Check ROS2 timestamps

```bash
ros2 topic echo /rosout --once 
```

### Check if simulation clock is publishing (Isaac Sim only)

```bash
ros2 topic echo /clock --once
```