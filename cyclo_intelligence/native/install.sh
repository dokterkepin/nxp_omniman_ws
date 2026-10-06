#!/usr/bin/env bash
# Prepare Cyclo Intelligence (src/cyclo_intelligence) to run natively - the
# parts colcon does not do. Cyclo's ROS packages build with the workspace:
#   colcon build --symlink-install
#
# Into nxp_omniman_ws/cyclo/ (outside git):
#   venv/        Cyclo's Python packages (CPU torch: only its dataset converter
#                uses it); numpy<2 as in Cyclo's image. Its ROS nodes run on
#                the system Python and see these through PYTHONPATH, set by
#                omniman_cyclo_bringup.launch.py for Cyclo's processes only.
#   workspace/   Cyclo's data: recordings, datasets, models, BT trees
# the web UI: npm build in src/cyclo_intelligence/orchestrator/ui (git-ignored),
# and the conda env cyclo_lerobot for Cyclo's LeRobot policy backend (policy
# runtime, training): Python 3.12, Cyclo's LeRobot 0.6 fork and its Zenoh SDK,
# at the commits Cyclo 1.4.0's lerobot image uses.
#
# Needs Node >= 22 (Cyclo's version). Safe to run again.
#
# Cyclo writes its data to /workspace (hard-coded). Point it here, once:
#   sudo ln -sfn ~/workspaces/nxp_omniman_ws/cyclo/workspace /workspace

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_CYCLO="git+https://github.com/ROBOTIS-GIT/lerobot-cyclo@240b4a0314ae0879cdd928c7f4bdc1eee9a01b3b"
ZENOH_ROS2_SDK="git+https://github.com/ROBOTIS-GIT/zenoh_ros2_sdk@v0.1.8"
CONDA="${CONDA:-$HOME/miniconda3/bin/conda}"
LEROBOT_ENV="${CYCLO_LEROBOT_ENV:-$HOME/miniconda3/envs/cyclo_lerobot}"
CYCLO_DIR="$(cd "$HERE/.." && pwd)"
SRC="$(dirname "$CYCLO_DIR")"
CYCLO_HOME="${CYCLO_HOME:-$(dirname "$SRC")/cyclo}"
say() { echo "[cyclo native] $*"; }

# ---- Python packages -----------------------------------------------------------
mkdir -p "$CYCLO_HOME"
touch "$CYCLO_HOME/COLCON_IGNORE"     # it is inside the workspace: keep colcon out of the venv
if [ ! -d "$CYCLO_HOME/venv" ]; then
    python3 -m venv --system-site-packages "$CYCLO_HOME/venv"
fi
say "installing Python packages into $CYCLO_HOME/venv"
"$CYCLO_HOME/venv/bin/pip" install -q --upgrade pip
"$CYCLO_HOME/venv/bin/pip" install -q 'numpy<2' fastapi 'uvicorn[standard]' 'pydantic>=2.6' \
    'docker>=6' huggingface_hub mcap mcap-ros2-support 'pyarrow==24.0.0' tqdm pandas \
    matplotlib psutil httpx websockets
"$CYCLO_HOME/venv/bin/pip" install -q torch --index-url https://download.pytorch.org/whl/cpu

# ---- data ----------------------------------------------------------------------
mkdir -p "$CYCLO_HOME/workspace/bt/trees" "$CYCLO_HOME/workspace/model/lerobot" \
         "$CYCLO_HOME/workspace/rosbag2" "$CYCLO_HOME/workspace/navigation"

# ---- LeRobot policy backend ----------------------------------------------------
if [ ! -x "$LEROBOT_ENV/bin/python" ]; then
    say "creating conda env $LEROBOT_ENV"
    "$CONDA" create -y -q -p "$LEROBOT_ENV" python=3.12
fi
say "installing LeRobot 0.6 (Cyclo's fork) into $LEROBOT_ENV"
"$LEROBOT_ENV/bin/pip" install -q "lerobot[dataset] @ $LEROBOT_CYCLO"
# eclipse-zenoh must match rmw_zenoh's zenoh-c (1.6.x), as in Cyclo's image.
"$LEROBOT_ENV/bin/pip" install -q "eclipse-zenoh>=1.6.0,<1.7.0" "rosbags>=0.11.0" \
    "GitPython>=3.1.18" "json5>=0.9.14" pyyaml "zenoh-ros2-sdk @ $ZENOH_ROS2_SDK"

# ROS message definitions for the Zenoh SDK, as Cyclo's init_zenoh_cache.sh
# provides them to its containers (the SDK's own fetch targets a branch that
# no longer exists upstream, and hangs).
ZENOH_CACHE="$HOME/.cache/zenoh_ros2_sdk"
mkdir -p "$ZENOH_CACHE"
for repo in common_interfaces rcl_interfaces; do
    if [ -d "$ZENOH_CACHE/$repo/.git" ]; then
        git -C "$ZENOH_CACHE/$repo" fetch -q --depth 1 origin jazzy
        git -C "$ZENOH_CACHE/$repo" reset -q --hard origin/jazzy
    else
        git clone -q --depth 1 --branch jazzy "https://github.com/ros2/$repo.git" "$ZENOH_CACHE/$repo"
    fi
done

# ---- web UI --------------------------------------------------------------------
say "building the web UI ($(node --version))"
(cd "$CYCLO_DIR/orchestrator/ui" && npm ci --legacy-peer-deps && npm run build)

say "done - build the workspace (colcon build --symlink-install), then"
say "       ros2 launch orchestrator omniman_cyclo_bringup.launch.py"
if [ "$(readlink -f /workspace 2>/dev/null)" != "$(readlink -f "$CYCLO_HOME/workspace")" ]; then
    say "still needed once: sudo ln -sfn $CYCLO_HOME/workspace /workspace"
fi
