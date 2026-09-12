# Navigation

## SLAM (Mapping)

Build a map of the environment using `slam_toolbox`, with wheel odometry from
`mecanum_drive_controller`. ros2_control must be running first (it also starts
the RPLidar and publishes `/scan`).

```bash
ros2 launch omniman_navigation slam_launch.py
```

This starts:
- **slam_toolbox** — builds the map
- **RViz** — visualization

Odometry comes from `mecanum_drive_controller`, which publishes
`odom -> base_footprint` itself (`enable_odom_tf: true` in `controllers.yaml`).
`rf2o_laser_odometry` and the `robot_localization` EKF were removed — the
mecanum controller is now the sole odometry source.

Drive the robot around using joystick or `cmd_vel` to build the map, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/workspaces/nxp_omniman_ws/src/omniman_navigation/maps/my_map
```

![SLAM Occupancy Grid Mapping](images/occupancy_grid_mapping.png)

---

## Nav2 (Autonomous Navigation)

Autonomous navigation using a saved map.
ros2_control must be running first.

```bash
ros2 launch omniman_navigation nav2_launch.py
```

This starts the full Nav2 stack:
- **map_server** + **AMCL** — localization on the saved map (`map -> odom`)
- **planner_server** — global planning with `nav2_navfn_planner::NavfnPlanner`
- **controller_server** — `RotationShimController` wrapping **MPPI**: it rotates
  in place to face the path first, then hands over to MPPI
- **filter_mask_server** + **costmap_filter_info_server** — keepout zones
- **behavior_server** — spin / backup / wait recoveries

Odometry is wheel-only, straight from `mecanum_drive_controller`
(`odom -> base_footprint`). There is no rf2o and no EKF.

In RViz:
1. Set the initial pose with **2D Pose Estimate** — AMCL publishes nothing until
   you do, so `map -> odom` is missing and the TF tree stays broken
2. Send goals with **2D Goal Pose**

> **Note:** Nav2's controller sends `geometry_msgs/TwistStamped`, but teleop_twist_joy sends
> plain `Twist`. A relay node (`twist_to_twist_stamped.py`) bridges this gap when user would like to teleoperate with joystick (use_joy:=true).

![Nav2 Navigation](images/nav2.png)

### Reading the costmap colours in RViz

RViz paints a costmap with one colour per cost value (0-254, plus 255 for
unknown). The exact palette is in `rviz_default_plugins` (`palette_builder.cpp`).

| Colour | Cost | Meaning | What it does to planning |
|---|---|---|---|
| nothing (see-through) | 0 | Free space | Path may go anywhere here |
| Blue -> purple -> red | 1-98 | Inflation: cost rising as you get nearer an obstacle | Legal, but the planner pays to go there, so it hugs the blue side |
| **Cyan** | 99 | Inscribed: robot centre here means the footprint touches the obstacle | Treated as blocked |
| **Magenta** | 100 | Lethal: the obstacle itself | Blocked |
| Grey-green | 255 (-1) | Unknown, never observed | Blocked unless the map says otherwise |
| Green | 101-127 | Invalid positive value | Should never appear - bad data |
| Red to yellow | 128-254 | Invalid negative value | Should never appear - bad data |

The underlying static map uses a different scheme: **white** free, **black**
occupied, **grey** unknown.

Faded, washed-out versions of the same colours are the *other* costmap drawn
underneath at lower alpha. Two costmaps overlap around the robot, so the vivid
patch is the local one and the pale wash is the global one.

Blue is cheap, red is expensive, and both are still drivable. Only cyan,
magenta and unknown actually block a path. If a goal lands on cyan or magenta,
Nav2 rejects it.

### Global vs local costmap

|  | Global costmap | Local costmap |
|---|---|---|
| Topic | `/global_costmap/costmap` | `/local_costmap/costmap` |
| Question it answers | "Which way round the building?" | "What is right in front of me now?" |
| Used by | `planner_server` - the whole path | `controller_server` (MPPI) - the next few moves |
| Frame | `map` | `odom` |
| Size | the whole saved map | 3 x 3 m window that travels with the robot |
| Update / publish | 2 Hz / 2 Hz | 10 Hz / 5 Hz |
| Layers | static + obstacle + keepout + inflation | obstacle + keepout + inflation |
| Inflation | radius 0.25 m, scaling 3.0 | radius 0.30 m, scaling 8.0 |

The local costmap has no static layer, so it only knows what the lidar sees
right now - that is what lets it dodge a person who was not on the map. The
global costmap starts from the saved map, so it can plan a route through rooms
the lidar cannot currently see.

Two things this explains in RViz:
- The local costmap is a **square that slides along with the robot, tilted
  relative to the map**. It lives in `odom`, and `odom` drifts away from `map`
  over time, so the tilt is normal.
- The local costmap is inflated *more* aggressively (0.30 m, scaling 8.0) than
  the global one, so a corridor can look passable in the global costmap and
  nearly closed in the local one.


### Control from a browser / iPad (Foxglove)

`nav2_launch.py` also starts `foxglove_bridge` on port 8765 (turn it off with
`foxglove:=false`). Install it once on the robot PC:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

On the iPad, open [app.foxglove.dev](https://app.foxglove.dev), choose
**Open connection → Foxglove WebSocket**, and enter `ws://192.168.51.151:8765`.

Panel setup:
1. **3D** panel: set the display frame to `map`, enable `/map`, `/scan`,
   `/plan` and the costmaps.
2. In the 3D panel settings under **Publish**, set the pose topic to
   `/goal_pose` and the pose estimate topic to `/initialpose`. Then use the
   toolbar buttons the same way as RViz's **2D Pose Estimate** and **2D Goal Pose**.
3. **Teleop** panel: topic `/cmd_vel`, for manual driving.

Save it as a layout so you only do this once.

> **If Safari won't connect:** app.foxglove.dev is served over `https`, and
> Safari can refuse a plain `ws://` connection from an `https` page. The fix is
> to serve the viewer over `http` on the LAN with Lichtblick (the open-source
> Foxglove fork, works with the same bridge):
> `docker run -d -p 8080:8080 ghcr.io/lichtblick-suite/lichtblick:latest`,
> then open `http://<pc-ip>:8080` on the iPad.

---

## Multi-Machine Setup

You can split the workload across two PCs over the same `ROS_DOMAIN_ID`.
The robot PC runs all hardware and navigation; the remote PC handles visualization and input.

**Robot PC:**
```bash
ros2 launch omniman_navigation slam_launch.py
```

**Remote PC (RViz):**
```bash
ros2 launch omniman_navigation rviz_launch.py
```

See [remote-access.md](remote-access.md) for SSH and DDS domain setup.

### Clock sync (Dual Machine, Nav2 Acceleration on the other machine)

Both PCs must agree on the time, or TF lookups fail and you get:

```
Message Filter dropping message: frame 'lidar_link' ...
'the timestamp on the message is earlier than all the data in the transform cache'
```

`timedatectl` saying "synchronized" on both is **not** enough — they can each be
synced to a different internet server and still be 300 ms apart. What matters is
that they agree with *each other*, so sync the remote PC to the robot.

**Robot PC** — serve time to the LAN:

```bash
sudo apt install chrony
```

Add to `/etc/chrony/chrony.conf`:

```
allow 192.168.51.0/24    # let the LAN ask us for time
local stratum 10         # keep serving even with no internet
```

```bash
sudo systemctl restart chrony && sudo systemctl enable chrony
```

**Remote PC** — sync to the robot:

```bash
sudo tee /etc/chrony/conf.d/robot.conf > /dev/null <<'EOF'
server 192.168.51.151 iburst prefer
EOF
sudo systemctl restart chrony
```

**Verify** (allow ~30 s):

```bash
chronyc sources     # robot should be marked ^*  (selected source)
chronyc tracking    # "Last offset" should be well under 1 ms
```
