# NXP Omniman System Overview

## Hardware
- **Mobile base**: Mecanum wheel robot (4 wheels, each with 9 passive rollers)
- **Arm**: 6-DOF manipulator (shoulder_yaw, upper_shoulder_pitch, arm_yaw, forearm_pitch, wrist_pitch, palm_yaw) + gripper (left/right finger prismatic, mimic joint)
- **Sensors**: RPLidar (lidar_link), USB camera, Dynamixel motors (x3 for gripper/wrist)
- **Compute**: NUC (erc-i3, hostname: erci3-NUC13ANH-B) + Dev machine (dokterkepin)

## Network / ROS2 Setup
- Both machines on same network, same `ROS_DOMAIN_ID=90`
- RMW: `rmw_cyclonedds_cpp`
- **NUC runs**: `nxp_omniman_launch.py` only (ros2_control, hardware, controllers, rplidar, usb_cam, joystick)
- **Dev machine runs**: Nav2, MoveIt, RViz, EKF, rf2o

## Launch Files
| File | Machine | Purpose |
|------|---------|---------|
| `omniman_ros2_control/launch/nxp_omniman_launch.py` | NUC | Hardware control, controllers, sensors |
| `omniman_navigation/launch/nav2_launch.py` | Dev | Navigation (AMCL, Nav2, EKF, rf2o) |
| `omniman_navigation/launch/slam_launch.py` | Dev | SLAM mapping (slam_toolbox, EKF, rf2o) |

## TF Tree
```
map → odom → base_footprint → base_link → [wheels, arm chain, lidar, camera]
     (AMCL)  (EKF)           (fixed)
```
- `map → odom`: published by AMCL (only when nav2_launch.py running)
- `odom → base_footprint`: published by EKF (robot_localization)
- `base_footprint → base_link → ...`: published by robot_state_publisher

## Odometry Chain
```
wheel encoders → /mecanum_drive_controller/odometry ─┐
                                                       ├→ EKF → /odometry/filtered + odom→base_footprint TF
lidar → rf2o → /odom_rf2o (zero covariance, normal) ──┘
```
- EKF publishes to `/odometry/filtered` (NOT `/odom`)
- Nav2 is configured to use `/odometry/filtered` via `odom_topic` in `nav2_params.yaml`
- `/odom` topic with 0 publishers is a stale DDS ghost — safe to ignore
- rf2o always outputs zero covariance — this is normal for rf2o package
- `odom1_relative: true` in `ekf.yaml` tells EKF to treat rf2o as incremental input

## Controllers (on NUC via ros2_control)
- `joint_state_broadcaster` — publishes /joint_states
- `mecanum_drive_controller` — wheel control, publishes /mecanum_drive_controller/odometry
- `arm_group_position_controller` — used with MoveIt Servo (use_servo:=true)
- `arm_controller` — JointTrajectoryController for MoveIt planning (use_servo:=false)
- `gripper_controller` — GripperActionController

## MoveIt Config (moveit_config package)
- SRDF: `nxp_omniman.srdf`
- Planning groups: `arm` (6 joints), `gripper` (left_finger only)
- Passive joints: 4 wheel joints + `right_finger_prismatic_joint` (mimic of left)
- End effector: `ee` at `ee_link`

### Virtual Joint Modes
| Mode | SRDF | RViz Fixed Frame | Requires |
|------|------|-----------------|---------|
| Arm only (standalone) | `fixed`, parent=`base_footprint`, child=`base_link` | `base_footprint` | Nothing |
| Mobile manipulator | `planar`, parent=`map`, child=`base_footprint` | `map` | Nav2 running |

**Current config**: `fixed` (arm-only mode)

## Known Issues / Gotchas

### DDS Ghost Nodes
- `ros2 node list` may show duplicate `/rf2o_laser_odometry` or `/dynamixel_hardware_interface`
- Always verify with `ps aux | grep <name>` — if only one process, it's a ghost
- Ghosts are stale DDS cache entries, harmless, disappear after timeout or reboot
- `/dynamixel_hardware_interface` appearing 3x is NORMAL — one per Dynamixel motor

### enable_odom_tf
- `mecanum_drive_controller` has `enable_odom_tf: false` in `controllers.yaml`
- Even with `true`, the controller does NOT publish `odom→base_footprint` TF (bug/limitation)
- TF is handled by EKF instead — do not change this setting

### URDF Issues Fixed
- `omniman.urdf`: `ee_joint` origin had comma-separated xyz (`0.000, 0.000, 0.080`) — fixed to space-separated

### usb_cam
- Camera is on NUC at `/dev/video0`
- Topics: `/image_raw`, `/camera_info`, `/image_raw/compressed`
- Node runs on NUC, topics visible on dev machine via DDS

## Useful Diagnostic Commands
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify odom TF exists
ros2 run tf2_ros tf2_echo odom base_footprint

# Check who publishes/subscribes a topic
ros2 topic info /topic_name -v

# Verify a process is actually running (vs DDS ghost)
ps aux | grep <process_name>

# Clear DDS ghost cache
ros2 daemon stop && rm -rf /dev/shm/cyclone* && ros2 daemon start

# Check controller states
ros2 control list_controllers

# Check EKF output
ros2 topic echo /odometry/filtered --once
```
