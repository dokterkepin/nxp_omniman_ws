# 🦾 Omniman single-arm adaptation

> This fork has been adapted from the original **dual-arm Panda** demo (below) to
> drive the **single 6-DOF omniman arm** with one hand. It reuses the omniman
> `moveit_config` (URDF/SRDF/kinematics) and the `moveit_servo` `PoseTracking`
> interface that are already in this workspace.

**What changed vs. the original**

| | Original | Omniman |
|---|---|---|
| Arms | 2 (left/right Panda) | 1 (`arm` group, `ee_link`) |
| Hand | both hands | **right hand only** |
| Planning frame | `world`/`panda_link0` | `base_link` |
| Arm command out | `*_arm_controller` | `/arm_controller/joint_trajectory` |
| Gripper | `JointTrajectory` | **`GripperCommand` action** (`/gripper_controller/gripper_cmd`) |
| Camera | `/dev/video0` | `/dev/video2` (C920) — `camera_device` param |
| Orientation | mapped from hand | **position-only by default** (6-DOF arm has no null space) |

**Files:** `src/omniman_pose_tracking_node.cpp`,
`scripts/hand_pose_publisher_node.py`,
`config/pose_tracking/omniman_pose_tracking.yaml`,
`launch/omniman_hand_teleop.launch.py`.

### Run it (real hardware)

The launch file starts **only** the servo + MediaPipe nodes. Bring up the robot
first, in **servo mode** — `arm_controller` (JointTrajectoryController) and
`gripper_controller` active, publishing `/joint_states`, and **no `move_group`
running** (the servo node owns the planning scene).

```bash
# Terminal 1 — your robot bringup in servo mode (controllers + /joint_states up)
#   e.g. ros2 launch omniman_ros2_control nxp_omniman_launch.py use_servo:=true

# Terminal 2 — build + launch hand teleop
cd ~/workspaces/nxp_omniman_ws
colcon build --packages-select mediapipe_dual_arm_control
source install/setup.bash
pip3 install -r src/mediapipe_dual_arm_control/requirements.txt   # first time only
ros2 launch mediapipe_dual_arm_control omniman_hand_teleop.launch.py camera_device:=2
```

Show your **right hand** to the camera: hand position moves the EE, raise **2+
fingers to OPEN** the gripper (fewer to close). Press `q` in the camera window to quit.

**Tuning** (no rebuild needed — all ROS params):
- Workspace box: `robot_x_min/max`, `robot_y_min/max`, `robot_z_min/max` on `hand_pose_publisher_node`.
- Hand depth → X mapping: `depth_near`, `depth_far`.
- Gripper travel: `gripper_open_position` (0.019), `gripper_close_position` (-0.010).
- Enable hand-orientation tracking: `ros2 launch ... track_orientation:=true` (expect EE-position drift on this arm).

> ⚠️ The defaults assume the omniman arm starts in a raised pose (e.g. the SRDF
> `ready` state). Start with small hand motions near the centre of the box.

---

# 🤖 Dual-Arm Teleoperation with MediaPipe Hand Pose Tracking (original)
Control two robotic arms using your hands! This project combines [MediaPipe](https://github.com/google-ai-edge/mediapipe) hand tracking with [MoveIt Servo](https://moveit.ai/moveit/ros2/servo/jog/2020/09/09/moveit2-servo.html) for smooth, intuitive dual-arm panda robot control.

## 🎬 Demo Video



https://github.com/user-attachments/assets/a066718c-373d-490d-a892-551655c1226c



*The video above shows dual-arm hand tracking and robot control in action. MediaPipe detects hand poses, which are mapped to robot arm movements using MoveIt Servo. Both arms and grippers respond to natural hand gestures.*

## ✨ Features

- Real-time hand detection (MediaPipe)
- Simultaneous dual-arm control in 3D space
- Gesture-based gripper control
- Smooth, responsive motion (MoveIt Servo)


## 🏁 Installation Options

You can run this project either **natively** on Ubuntu or inside a **Docker container**. Choose the method that best fits your needs:


### 🐧 Native (Recommended for development)

1. **Install MoveIt 2, controllers, and Panda description dependencies:**
   ```bash
   sudo apt install \
     ros-humble-moveit \
     ros-humble-moveit-servo \
     ros-humble-moveit-visual-tools \
     ros-humble-ros2-control \
     ros-humble-ros2-controllers \
     ros-humble-moveit-resources-panda-description
   ```

2. **Create and source a ROS 2 workspace (if you don't have one):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   ```

3. **Clone this repo in the `src` folder and build:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Nabil-Miri/mediapipe_dual_arm_control.git
   cd ..
   colcon build
   ```

4. **Install Python dependencies:**
   ```bash
   pip3 install -r src/mediapipe_dual_arm_control/requirements.txt
   ```

5. **Launch the system:**
   ```bash
   # Terminal 1: Source ROS and launch robot system
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch mediapipe_dual_arm_control dual_arm_teleop.launch.py

   # Terminal 2: Source ROS and start hand tracking
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 src/mediapipe_dual_arm_control/scripts/hand_pose_publisher_node.py
   ```

---

### 🐳 Docker (Recommended for easy setup/reproducibility)

1. **Build the Docker image** (from the project root):
   ```bash
   docker build -f docker/Dockerfile -t dual-arm-teleop:humble .
   ```

2. **Run the Docker container:**
   ```bash
   xhost +local:root
   docker run -it --rm \
       --name dual-arm-teleop-container \
       --net=host \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --device=/dev/video0 \
       dual-arm-teleop:humble
   ```
   - Adjust `--device=/dev/video0` if your camera is on a different device.

3. **Open a second terminal in the running container:**
   ```bash
   docker exec -it dual-arm-teleop-container bash
   ```

4. **Launch the system inside the container:**
   ```bash
   # Terminal 1: Launch robot system
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch mediapipe_dual_arm_control dual_arm_teleop.launch.py

   # Terminal 2: Start hand tracking
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 src/mediapipe_dual_arm_control/scripts/hand_pose_publisher_node.py
   ```

## 🛠️ How It Works

- **Python node**: Tracks hands, counts fingers, publishes target poses and gripper commands for each arm
- **C++ nodes**: Subscribe to hand poses, validate workspace, control each arm and gripper independently using MoveIt Servo
- **ROS2 topics**: `/left_hand_target_pose`, `/right_hand_target_pose`, `/left_hand_gripper_control`, `/right_hand_gripper_control`

## 📋 Requirements

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.8+
- Camera

## 📁 Repository Structure

```
mediapipe_dual_arm_control/      # Main ROS2 package
├── scripts/
│   └── mediapipe_dual_arm_coordinator.py      # Python hand tracking node
├── src/
│   └── dual_arm_servo_node.cpp                # C++ dual-arm servo node
├── launch/
│   └── dual_arm_teleop.launch.py             # Main launch file
├── config/
│   ├── robot/         # Robot URDF, SRDF, controllers, kinematics, etc.
│   ├── planning/      # Planning configs
│   ├── pose_tracking/ # Pose tracking configs
│   ├── rviz/          # RViz config
│   └── hand/          # Hand xacro
├── requirements.txt                           # Python dependencies
├── docker/
│   └── Dockerfile                             # Docker setup for reproducibility
└── ...                                        # Other package files
```

## 🐞 Troubleshooting

- **Camera not detected**: Check permissions/connections
- **Robot not moving**: Verify controllers are loaded
- **Build fails**: Try `--parallel-workers 1` and ensure enough RAM
- **ROS2 not sourced**: Run `source /opt/ros/humble/setup.bash`

---

<!-- ## 🏗️ System Architecture

<details>
<summary><strong>System Architecture Details (click to expand)</strong></summary>

#### 🐍 Python Hand Tracking Node (`mediapipe_dual_arm_coordinator.py`)
The vision processing powerhouse that:
- **Captures camera feed** at 50Hz using OpenCV
- **Detects hands** using Google's MediaPipe library
- **Maps hand positions** from camera coordinates to robot workspace
- **Counts fingers** to determine gripper open/close commands
- **Publishes to ROS2 topics** for each arm independently

#### ⚡ C++ Dual Arm Servo Node (`dual_arm_servo_node.cpp`)
Two instances of this node run simultaneously (one per arm):
- **Subscribes to hand poses** from the Python node
- **Validates workspace limits** for safety
- **Controls MoveIt Servo** for real-time arm movement
- **Manages gripper trajectories** via joint controllers
- **Runs in separate namespaces** to prevent conflicts

#### 🔄 Data Flow
1. **Camera** → Python node detects hands
2. **Python node** → Publishes `/left_hand_target_pose` and `/right_hand_target_pose`
3. **C++ nodes** (`dual_arm_servo_node.cpp`) → Subscribe to poses and validate workspace
4. **MoveIt Servo** → Converts poses to joint commands
5. **Controllers** → Move robot arms and grippers

</details>

--- -->

## 📡 ROS2 Topics

| Topic | Description |
|-------|-------------|
| `/left_hand_target_pose` | Target position for left arm |
| `/right_hand_target_pose` | Target position for right arm |
| `/left_hand_gripper_control` | Open/close left gripper |
| `/right_hand_gripper_control` | Open/close right gripper |

<!-- ## 🛠️ Development Journey 

<details>
<summary><strong>From Single Arm to Dual Arm Control</strong></summary>

**Initial Approach:**
- MediaPipe detected hand landmarks
- Converted hand position to robot pose
- Sent pose commands directly to the robot

**❌ Problem**: This worked but the robot was planning in a weird way - it would plan full trajectories for every small hand movement, making the control jerky and unnatural.

### 💡 Discovery: MoveIt Servo
Upon reading more about real-time robot control, I discovered **MoveIt Servo** - a component designed specifically for real-time, smooth robot control. MoveIt Servo:
- Accepts continuous pose or twist commands
- Performs real-time collision checking
- Provides smooth, responsive robot motion
- Integrates seamlessly with MoveIt's planning framework

This was exactly what I needed for natural hand-to-robot control!

### 🔥 Scaling to Dual Arms
Once single arm control worked, I moved to dual arms:

#### 🤖 Dual Arm Moveit Configs Source

To set up the dual-arm robot, I sourced the URDF from the `moveit_resources` package. However, the dual-arm Panda URDF was only available in the `ros2` branch of `moveit_resources` and not in the `humble` branch. As a result, I had to adapt and modify the URDF to ensure compatibility with ROS 2 Humble and my workspace.

Additionally, I added a controller for the gripper, as the default dual-arm MoveIt resources did not provide one. This ensures both arms and grippers can be controlled independently in the simulation and with MoveIt Servo. You must add the following controller definitions to your `ros2_controllers.yaml`:

```yaml
left_panda_fingers_controller:
  type: joint_trajectory_controller/JointTrajectoryController
right_panda_fingers_controller:
  type: joint_trajectory_controller/JointTrajectoryController
```

<details>
<summary><strong>⚠️ Note on ompl_planning.yaml Format (click to expand)</strong></summary>

If you encounter errors related to `request_adapters` or `response_adapters` in `ompl_planning.yaml`, make sure these fields are formatted as multi-line strings (not as YAML arrays). Use the `>-` syntax as shown below:

```yaml
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planning_request_adapters/ResolveConstraintFrames;
  default_planning_request_adapters/ValidateWorkspaceBounds;
  default_planning_request_adapters/CheckStartStateBounds;
  default_planning_request_adapters/CheckStartStateCollision

response_adapters: >-
  default_planning_response_adapters/AddTimeOptimalParameterization
  default_planning_response_adapters/ValidateSolution
  default_planning_response_adapters/DisplayMotionPath
```

This is required for MoveIt 2 to parse the adapters correctly. If you see errors about adapters, check that your YAML matches this format.

</details>

#### 🚫 The Dual Control Problem
**⚠️ Major Issue**: I could only control one robot arm at a time, not both simultaneously.

**Root Cause**: Both C++ nodes were trying to use the same:
- Topic names (causing conflicts)
- MoveIt planning groups (interference)
- Servo instances (resource competition)

**Solution Strategy**:
1. **Namespace Isolation**: Each arm runs in its own namespace (`/left_arm`, `/right_arm`)
2. **Separate Parameter Files**: Each arm loads its own servo configuration
3. **Independent Servo Instances**: Each C++ node creates its own MoveIt Servo
4. **Unique Topic Names**: MediaPipe publishes to arm-specific topics

### 🎉 Final Result
A fully functional dual arm hand tracking system where:
- Both arms can be controlled simultaneously and independently
- Smooth, real-time motion using MoveIt Servo
- Finger counting controls grippers on both arms
- No interference between left and right arm control
-->
## 🚀 Potential Improvements

- **Dual-Arm Robot**: Use a dual arm robot like [FR3 DUO](https://franka.de/news/real-world-robotics-insights-from-fr3-duo-mobile-showcase) rather than 2 separate manipulators.
- **Gesture Start/Stop**: Use hand gestures to start or stop the system.
- **Better Depth**: Add monocular depth estimation for more accurate hand positions.
- **Smoother Motion**: Improve filtering for steadier robot vision pose targets.

</details>

---


## 🤝 Contributing

Feel free to fork, modify, and submit PRs! Suggestions and improvements are welcome.
