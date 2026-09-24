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

---

## Process & Thread Debugging

### `ps aux` — what is running

```bash
ps aux                      # everything
ps aux | grep [r]viz        # brackets stop grep matching itself
ps aux --sort=-%mem | head  # biggest memory users
ps aux | cut -c1-95         # trim runaway command lines
```

The three letters: `a` = all users, `u` = detailed format, `x` = **include
processes with no terminal** (daemons, background nodes).

Columns:

| column | meaning |
|---|---|
| `%CPU` | **average since the process started**, NOT current load |
| `VSZ`  | virtual address space - often absurd, mostly ignore |
| `RSS`  | resident memory in KB - the real RAM number |
| `TTY`  | `?` = no terminal (daemon), `pts/0` = started from a terminal |
| `STAT` | state + modifiers, see below |
| `TIME` | total CPU time consumed |

For *current* CPU use `top`, `htop`, or `pidstat 1`.

### STAT codes

```
S  sleeping (normal)     R  running      D  uninterruptible I/O (cannot be killed)
Z  zombie                T  stopped
```

Modifiers stack on: `N` low priority, `<` high priority, `L` **pages locked in
memory (mlockall)**, `l` multi-threaded, `s` session leader, `+` foreground.

`ros2_control_node` shows `SNLl+`. That `L` confirms `memlock` is working - the
control loop has its memory pinned and cannot be swapped out.

### Threads (what `ps aux` cannot show)

`ros2_control`'s realtime loop is a **thread**, so the process shows
`SCHED_OTHER` and tells you nothing. Use `-L`:

```bash
# threads of one process
ps -Lo pid,tid,rtprio,cls,psr,comm -p $(pgrep -f ros2_control_node | head -1)

# every realtime thread on the machine (-e = all processes)
ps -eLo pid,tid,rtprio,cls,psr,comm --sort=-rtprio | awk 'NR==1 || $4!="TS"'
```

`rtprio` = realtime priority, `cls` = scheduling class (`FF` = SCHED_FIFO,
`TS` = normal), `psr` = which CPU core. Expect one line with `50 FF`.

`-e` and `-p` are alternatives (all vs one). `-L` turns processes into threads.

### Finding and killing

```bash
pgrep -af aruco             # -a shows full command line, -f matches it too
kill -INT <PID>             # kill takes NUMBERS
pkill -INT -f <pattern>     # pkill takes NAMES/patterns
```

**`kill` takes PIDs, `pkill` takes names.** `pkill 19038` looks for a process
*named* "19038", finds nothing, and silently does nothing.

`-f` is needed for ROS nodes because they run from long paths; without it
`pgrep` only sees the process name, truncated to 15 characters.

**Always preview with `pgrep` before `pkill`** - `pkill -f` matches anything
whose command line contains the pattern, including the shell you are typing in.

Signals:

```bash
pkill -INT -f "ros2 launch omniman_ros2_control"   # SIGINT = Ctrl-C, CLEAN
pkill -9   -f ros2_control_node                    # SIGKILL - NO cleanup
```

Signal the launch, not individual nodes - it propagates to every node it started.
Killing a process does NOT means stop the hardware, beaware of this and check if there is a service call too kill the hardware instead of using this

---

## Reading Old Node Logs

Every ROS 2 node writes what it logs to a file in `~/.ros/log/` - one file per
run. So a mission you ran this afternoon is still there, even after the
terminal is closed. (A normal terminal does NOT keep its output, and
`journalctl` only has systemd services - neither helps here.)

The file names are hard to read: `python3_263023_1790144702152.log` =
`<executable>_<pid>_<start time in ms since 1970>.log`, and every Python node
is called `python3`. So: first find the file, then open it.

### 1. Add `roslogs` to `~/.bashrc` (once)

```bash
roslogs() {   # roslogs [node] [how many]   e.g. roslogs pick_place_bt 20
  for f in $(ls -tr ~/.ros/log/*.log | tail -n ${2:-30}); do
    node=$(grep -om1 '\] \[[a-z_0-9]*\]' "$f" | tr -d '[] ')
    [ -n "$1" ] && [ "$node" != "$1" ] && continue
    ms=$(basename "$f" .log | grep -o '[0-9]*$')
    start=$(date -d @${ms:0:10} '+%a %m-%d %H:%M:%S')
    end=$(date -r "$f" '+%H:%M:%S')
    printf '%s -> %s  %-20s %s\n' "$start" "$end" "${node:-?}" "$(basename $f)"
  done
}
```

Then `source ~/.bashrc`.

### 2. Find the run

```bash
roslogs                                  # newest 30 log files, any node
roslogs pick_place_bt                    # only the mission, among the newest 30
roslogs pick_place_bt 3000               # look further back
roslogs pick_place_bt 3000 | grep " 09-23 1[2-5]:"    # one afternoon
```

```
Wed 09-23 14:25:02 -> 14:31:10  pick_place_bt        python3_263023_1790144702152.log
```

start date and time -> last line written, node, file name.

**Run not listed?** It only looks at the newest N files, and every node
writes one - raise the number (`3000`).

### 3. Show the log

```bash
less ~/.ros/log/python3_263023_1790144702152.log
```

| key | does |
|---|---|
| `G` / `g` | end (how it ended) / start |
| `/ERROR` then `n` | search, next match |
| `q` | quit |

Quick looks without opening it:

```bash
tail -30 ~/.ros/log/<file>.log                  # how it ended
grep -E "ERROR|WARN" ~/.ros/log/<file>.log      # only the problems
```

### 4. Nicer viewer: lnav (optional)

```bash
sudo apt install lnav
lnav ~/.ros/log/<file>.log                                  # one run
lnav $(grep -l "\[pick_place_bt\]" ~/.ros/log/*.log)        # every mission run
```

Colours, `e` / `E` = next / previous error, `q` = quit.

**Do not run `lnav ~/.ros/log/*.log`** - there are thousands of files and
lnav stops with "Too many open files", showing almost nothing. Always pick
the files first (as above).

Filter inside lnav: press `:` and type `filter-in <text>`, Enter. Remove it:
`TAB`, `D`, `TAB`.

### 5. Keep it tidy

```bash
find ~/.ros/log -name "*.log" -mtime +7 -delete     # delete logs older than a week
```

Or give every day its own folder - add to `~/.bashrc`:

```bash
export ROS_LOG_DIR=~/.ros/log/$(date +%Y-%m-%d)
```

### 6. Save output of anything else

For a command that is not a ROS node, save what it prints yourself:

```bash
some_command 2>&1 | tee ~/run_$(date +%m%d_%H%M).log
```
