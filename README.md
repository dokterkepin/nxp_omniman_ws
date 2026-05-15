# NXP Omniman

A ROS 2 Humble mobile manipulator featuring a mecanum-wheeled base with a 6-DOF arm and gripper.
The robot uses CyberGear motors (base + arm), Dynamixel servos (wrist + gripper), and an RPLidar
for SLAM and navigation.

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [CAN Bus Setup](#can-bus-setup)
- [Udev Rules](#udev-rules)
- [Build](#build)
- [ros2_control](#ros2_control)
- [MoveIt](#moveit)
- [Navigation](#navigation)
  - [SLAM (Mapping)](#slam-mapping)
  - [Nav2 (Autonomous Navigation)](#nav2-autonomous-navigation)
- [Simulation (Isaac Sim)](#simulation-isaac-sim)
  - [Why `use_sim` Matters](#why-use_sim-matters)
- [Package Overview](#package-overview)
- [Multi-Machine Setup](#multi-machine-setup)

## Hardware Overview

| Component | Hardware | Interface | Count |
|---|---|---|---|
| Mecanum wheels | CyberGear motors | CAN bus (`can_base`) | 4 |
| Arm joints (shoulder, upper shoulder, arm, forearm) | CyberGear motors | CAN bus (`can_arm`) | 4 |
| Wrist + gripper (wrist pitch, palm yaw, finger) | Dynamixel servos | Serial (`/dev/dynamixel`) | 3 |
| Lidar | RPLidar | Serial (`/dev/rplidar`) | 1 |

## Prerequisites

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble Hawksbill (desktop-full)
- **MoveIt 2:** Humble branch
- **Nav2:** Humble branch

Install ROS 2 Humble following the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

## Installation

```bash
# Create workspace
mkdir -p ~/workspaces/nxp_omniman_ws/src
cd ~/workspaces/nxp_omniman_ws/src

# Clone the repository
git clone git@github.com:dokterkepin/nxp_omniman_ws.git .

# Install all ROS dependencies via rosdep
cd ~/workspaces/nxp_omniman_ws
rosdep install --from-paths src --ignore-src -r -y

# Install system tools not covered by rosdep
sudo apt install -y can-utils
```

## CAN Bus Setup

The robot uses two CAN buses: `can_base` for mecanum wheels and `can_arm` for arm joints.

```bash
# Bring up both CAN interfaces (run once after boot)
cd ~/workspaces/nxp_omniman_ws/src/cybergear_hardware
sudo bash bringup_canbus.sh
```

Verify CAN is up:

```bash
ip link show can_base
ip link show can_arm
```

For more details, see [cybergear_hardware/about_can.md](cybergear_hardware/about_can.md) and
[cybergear_hardware/about_socketcan.md](cybergear_hardware/about_socketcan.md).

## Udev Rules

Udev rules map the RPLidar and Dynamixel to fixed device names (`/dev/rplidar`, `/dev/dynamixel`),
so the port doesn't change when devices are plugged in different USB slots.

See [dynamixel_hardware/about_udev.md](dynamixel_hardware/about_udev.md) for setup instructions.

## Build

```bash
cd ~/workspaces/nxp_omniman_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## ros2_control

This launches all hardware interfaces, the controller manager, and controllers
(mecanum drive, arm trajectory, gripper).

```bash
ros2 launch omniman_ros2_control nxp_omniman_launch.py
```

To also enable joystick teleoperation:

```bash
ros2 launch omniman_ros2_control nxp_omniman_launch.py use_joy:=true
```

Verify controllers are running:

```bash
ros2 control list_controllers
```

You should see:
- `joint_state_broadcaster` — active
- `mecanum_drive_controller` — active
- `arm_controller` — active
- `gripper_controller` — active

---

## MoveIt

MoveIt provides motion planning for the 6-DOF arm and gripper.
ros2_control must be running first.

```bash
ros2 launch omniman_moveit_config demo.launch.py
```

This opens RViz with the MoveIt motion planning plugin. You can:
- Drag the interactive marker to set a goal pose
- Click **Plan & Execute** to move the arm
- Use the gripper controls to open/close

![MoveIt RViz](docs/images/moveit.png)

---

## Navigation

### SLAM (Mapping)

Build a map of the environment using `slam_toolbox` with laser odometry from `rf2o`.
ros2_control must be running first.

```bash
ros2 launch omniman_navigation slam_launch.py
```

This starts:
- **RPLidar** — publishes `/scan`
- **rf2o_laser_odometry** — laser-based odometry → `/odom_rf2o`
- **EKF (robot_localization)** — fuses mecanum wheel odom + rf2o → `/odometry/filtered`
- **slam_toolbox** — builds the map
- **RViz** — visualization

Drive the robot around using joystick or `cmd_vel` to build the map, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/workspaces/nxp_omniman_ws/src/omniman_navigation/maps/my_map
```

![SLAM Occupancy Grid Mapping](docs/images/occupancy_grid_mapping.png)

### Nav2 (Autonomous Navigation)

Autonomous navigation using a saved map.
ros2_control must be running first.

```bash
ros2 launch omniman_navigation nav2_launch.py
```

This starts the full Nav2 stack:
- **map_server** + **AMCL** — localization on the saved map
- **Nav2 planner/controller** — path planning + DWB local planner (configured for mecanum)
- **rf2o + EKF** — odometry fusion

In RViz:
1. Set the initial pose with **2D Pose Estimate**
2. Send goals with **2D Goal Pose**

> **Note:** Nav2's DWB controller sends `geometry_msgs/TwistStamped`, but teleop_twist_joy sends
> plain `Twist`. A relay node (`twist_to_twist_stamped.py`) bridges this gap when needed.

![Nav2 Navigation](docs/images/nav2.png)

---

## Simulation (Isaac Sim)

The robot can run in NVIDIA Isaac Sim using the `TopicBasedSystem` ros2_control plugin.
**Always pass `use_sim:=true`** to every launch file when running in simulation.
For more detail [about use_sim:=true](about_use_sim.md)

```bash
ros2 launch omniman_ros2_control nxp_omniman_launch.py use_sim:=true use_joy:=true
ros2 launch omniman_moveit_config moveit_rviz.launch.py
ros2 launch omniman_navigation slam_launch.py use_sim:=true
```

The USD scene files are in `omniman_description/urdf/omniman_isaac/` (open `omniman_isaac.usd` in Isaac Sim).

![Isaac Sim](docs/images/isaacsim.png)

---

## Package Overview

| Package | Description |
|---|---|
| `omniman_description` | URDF/xacro model, meshes, and Isaac Sim USD files |
| `omniman_ros2_control` | Main launch file, controller config, URDF with ros2_control hardware tags |
| `omniman_moveit_config` | MoveIt configuration (SRDF, kinematics, planning pipeline) |
| `omniman_navigation` | SLAM, Nav2, EKF configs, and saved maps |
| `omniman_commander` | Mission scripts and MoveIt commander nodes (pick-place, FIRA, traffic) |
| `cybergear_hardware` | CyberGear/Robstride motor driver, CAN bus setup, ros2_control plugin |
| `dynamixel_hardware` | Dynamixel servo driver, SDK, and ros2_control plugin |
| `rf2o_laser_odometry` | Laser scan-based odometry (rf2o algorithm) |
| `rplidar_ros` | RPLidar driver |
