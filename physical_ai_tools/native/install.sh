#!/usr/bin/env bash
# Prepare physical_ai_tools (src/physical_ai_tools) to run natively, without
# Docker - the parts rosdep and colcon do not do. Its ROS packages build with
# the workspace:
#   colcon build --symlink-install
#
# This script sets up:
#   conda env lerobot_jazzy   Python 3.12 with PyTorch (CUDA), and the LeRobot
#                             that ships in physical_ai_tools/lerobot (0.2.0),
#                             installed editable - not the upstream release,
#                             which does not match. datasets is pinned to
#                             <=3.6.0 and numpy to <2, as in the Dockerfile.
#                             Training (lerobot.scripts.train) runs in it, and
#                             physical_ai_server imports it through PYTHONPATH.
#   physical_ai_manager       the web UI: npm install (git-ignored node_modules).
#
# Needs: Miniconda (~/miniconda3), Node >= 22 and the ROS packages that
# physical_ai_server runs with. It checks first and tells you what is missing;
# it never installs anything with sudo. Safe to run again.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS="$(cd "$HERE/.." && pwd)"
CONDA="${CONDA:-$HOME/miniconda3/bin/conda}"
ENV="${PHYSICAL_AI_ENV:-$HOME/miniconda3/envs/lerobot_jazzy}"
say() { echo "[physical_ai native] $*"; }

# ---- what must be there already -------------------------------------------------
missing=()
[ -x "$CONDA" ] || missing+=("conda: not at $CONDA - install Miniconda into ~/miniconda3, or set CONDA=/path/to/conda")
if command -v node >/dev/null 2>&1; then
    [ "$(node --version | sed 's/^v//; s/\..*//')" -ge 22 ] \
        || missing+=("Node >= 22 (found $(node --version)) - the web UI is built with Node 22")
else
    missing+=("Node >= 22 (not found) - e.g. nvm install 22")
fi
apt_missing=()
for pkg in ros-jazzy-rosbridge-suite ros-jazzy-web-video-server ros-jazzy-image-transport-plugins; do
    dpkg -s "$pkg" >/dev/null 2>&1 || apt_missing+=("$pkg")
done
[ ${#apt_missing[@]} -eq 0 ] \
    || missing+=("ROS packages: sudo apt update && sudo apt install ${apt_missing[*]}")
if [ ${#missing[@]} -gt 0 ]; then
    say "missing - install these first, then run this again:"
    printf '  - %s\n' "${missing[@]}"
    exit 1
fi

# ---- conda env with LeRobot ------------------------------------------------------
if [ ! -x "$ENV/bin/python" ]; then
    say "creating conda env $ENV"
    "$CONDA" create -y -q -p "$ENV" python=3.12
fi
if [ ! -x "$ENV/bin/pip" ]; then
    say "$ENV exists but has no pip - remove it (rm -rf $ENV) and run this again"
    exit 1
fi
say "installing PyTorch into $ENV"
"$ENV/bin/pip" install -q torch torchvision
say "installing LeRobot from $TOOLS/lerobot (editable)"
"$ENV/bin/pip" install -q -e "$TOOLS/lerobot[smolvla]"
"$ENV/bin/pip" install -q 'datasets>=2.19.0,<=3.6.0' 'numpy<2'
"$ENV/bin/python" - <<'EOF'
import lerobot, torch
print(f"[physical_ai native] lerobot {lerobot.__version__} from {lerobot.__file__}")
print(f"[physical_ai native] torch {torch.__version__}, CUDA available: {torch.cuda.is_available()}")
if not torch.cuda.is_available():
    print("[physical_ai native] WARNING: no CUDA - training and inference would run on the CPU")
EOF

# ---- web UI --------------------------------------------------------------------
say "installing the web UI's packages ($(node --version))"
(cd "$TOOLS/physical_ai_manager" && npm install)

say "done - build the workspace (colcon build --symlink-install), then:"
say "  export PYTHONPATH=$ENV/lib/python3.12/site-packages:$TOOLS/lerobot/src:\$PYTHONPATH"
say "  ros2 launch physical_ai_server physical_ai_server_bringup.launch.py"
say "  cd $TOOLS/physical_ai_manager && npm start     # http://localhost:3000"
