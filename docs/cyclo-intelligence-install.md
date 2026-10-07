# Installing Cyclo Intelligence on a fresh PC

Steps to get Cyclo Intelligence (`cyclo_intelligence/`) running natively,
with no Docker, on a new PC. Do this on the PC that runs Cyclo (UI,
recording, policy, training), not on the robot.

**Before you start:** the workspace is cloned, its dependencies are installed
and it builds, as in [README.md](../README.md) (Installation and Build), on
ROS 2 Jazzy. An NVIDIA GPU with its driver is needed for policies and
training.

## 1. Switch to the Cyclo branch

```bash
cd ~/workspaces/nxp_omniman_ws/src
git checkout cyclo_adapt
```

## 2. Packages Cyclo needs that rosdep does not install

```bash
sudo apt install python3-venv ros-jazzy-rmw-zenoh-cpp ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server ros-jazzy-rosbag2-storage-mcap ros-jazzy-nav2-msgs
```

## 3. Node.js 22 and Miniconda

The web UI is built with Node 22 or newer. The policy backend runs in a conda
env, and `install.sh` looks for conda in `~/miniconda3`.

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 22

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p ~/miniconda3
```

## 4. Use Zenoh

Cyclo talks to ROS over Zenoh. Add to `~/.bashrc`, then open a new terminal:

```bash
export ROS_DOMAIN_ID=67                    # the same as the robot
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

## 5. Build

```bash
cd ~/workspaces/nxp_omniman_ws
colcon build --symlink-install
```

Keep `--symlink-install`. omniman's behaviour-tree nodes are in subfolders
that Cyclo's `setup.py` does not list, and only this build finds them.

## 6. Run the installer

```bash
./src/cyclo_intelligence/native/install.sh
```

It takes a while, and it is safe to run again. It sets up:

| Where | What |
|---|---|
| `nxp_omniman_ws/cyclo/venv` | Python packages for Cyclo's ROS nodes |
| `nxp_omniman_ws/cyclo/workspace` | Cyclo's data: recordings, datasets, models, BT trees |
| conda env `cyclo_lerobot` | the policy backend: LeRobot 0.6 (Cyclo's fork) and its Zenoh SDK |
| `~/.cache/zenoh_ros2_sdk` | ROS message definitions for that SDK |
| `cyclo_intelligence/orchestrator/ui/build` | the web UI |

`nxp_omniman_ws/cyclo/` is outside git.

## 7. Link `/workspace` (once)

Cyclo reads and writes its data in `/workspace`:

```bash
sudo ln -sfn ~/workspaces/nxp_omniman_ws/cyclo/workspace /workspace
```

## 8. Check that it starts

```bash
# terminal 1: Zenoh router, before any ROS node.
# Connects to the robot's router; leave out ZENOH_CONFIG_OVERRIDE without the robot.
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.51.151:7447"]' \
    ros2 run rmw_zenoh_cpp rmw_zenohd

# terminal 2
ros2 launch orchestrator omniman_cyclo_bringup.launch.py
```

Open **http://localhost:7080**. The UI's Home page shows the robot type
(`omniman`). Ctrl+C in terminal 2 stops everything.

## 9. Models trained with physical_ai_tools

Policies trained with physical_ai_tools (LeRobot 0.2) must be converted once
before Cyclo (LeRobot 0.6) can load them:

```bash
~/miniconda3/envs/cyclo_lerobot/bin/python -m lerobot.processor.migrate_policy_normalization \
    --pretrained-path /workspace/model/lerobot/<model>/checkpoints/<step>/pretrained_model
```

Copy the model under `/workspace/model` first; the UI only loads from there.
Load the converted `pretrained_model_migrated` folder.

## If it does not work

| Message or symptom | Cause |
|---|---|
| `not installed yet - run install.sh` | step 6 did not finish; run it again |
| `Directory does not exist: /workspace/model` | step 7 is missing |
| no topics from the robot | the router (step 8) is not running, or `ROS_DOMAIN_ID` differs |
| `colcon build` fails with old paths | delete that package's folders in `build/` and `install/`, build again |
