#!/usr/bin/env bash
# Prepare Cyclo Intelligence (src/cyclo_intelligence) to run natively - the
# parts colcon does not do. Cyclo's ROS packages build with the workspace:
#   colcon build --symlink-install
#
# Into nxp_omniman_ws/cyclo/ (outside git):
#   venv/        Cyclo's Python packages (CPU torch: only its dataset converter
#                uses it); numpy<2 as in Cyclo's image. Its ROS nodes run on
#                the system Python and see these through PYTHONPATH, set by
#                cyclo_bringup.launch.py for Cyclo's processes only.
#   workspace/   Cyclo's data: recordings, datasets, models, BT trees
# and the web UI: npm build in src/cyclo_intelligence/orchestrator/ui (git-ignored).
#
# Needs Node >= 22 (Cyclo's version). Safe to run again.
#
# Cyclo writes its data to /workspace (hard-coded). Point it here, once:
#   sudo ln -sfn ~/workspaces/nxp_omniman_ws/cyclo/workspace /workspace

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"
CYCLO_DIR="$SRC/cyclo_intelligence"
CYCLO_HOME="${CYCLO_HOME:-$(dirname "$SRC")/cyclo}"
say() { echo "[omniman_cyclo] $*"; }

# ---- Python packages -----------------------------------------------------------
mkdir -p "$CYCLO_HOME"
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
for tree in "$SRC"/omniman_cyclo/trees/*.xml; do        # omniman's example trees
    ln -sfn "$tree" "$CYCLO_HOME/workspace/bt/trees/$(basename "$tree")"
done

# ---- web UI --------------------------------------------------------------------
say "building the web UI ($(node --version))"
(cd "$CYCLO_DIR/orchestrator/ui" && npm ci --legacy-peer-deps && npm run build)

say "done - build the workspace (colcon build --symlink-install), then"
say "       ros2 launch omniman_cyclo cyclo_bringup.launch.py"
if [ "$(readlink -f /workspace 2>/dev/null)" != "$(readlink -f "$CYCLO_HOME/workspace")" ]; then
    say "still needed once: sudo ln -sfn $CYCLO_HOME/workspace /workspace"
fi
