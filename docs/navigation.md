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
