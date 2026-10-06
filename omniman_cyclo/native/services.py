#!/usr/bin/env python3
"""
Cyclo's services as plain processes - what s6 does inside Cyclo's container,
for running Cyclo natively (omniman_cyclo cyclo_bringup.launch.py).

    orchestrator   ros2 launch orchestrator orchestrator_bringup.launch.py
                   (orchestrator, rosbridge, rosbag recorder, web_video_server)
    cyclo_data     ros2 run cyclo_data cyclo_data_node
    bt_node        ros2 launch orchestrator bt_node.launch.py robot_type:=<type>

Started and stopped by the UI's buttons through the supervisor
(supervisor_native.py), exactly as in the container. Each runs in its own
process group with the supervisor's environment; logs go to
<CYCLO_HOME>/log/services/<name>.log.
"""

import os
from pathlib import Path
import signal
import subprocess
import time

# Cyclo's runtime files, outside git: nxp_omniman_ws/cyclo (set by cyclo_bringup.launch.py).
CYCLO_HOME = Path(os.environ.get('CYCLO_HOME', Path(__file__).resolve().parents[3] / 'cyclo'))
RUN_DIR = CYCLO_HOME / 'run'
LOG_DIR = CYCLO_HOME / 'log' / 'services'
BT_ROBOT_TYPE_FILE = RUN_DIR / 'bt_node_robot_type'

COMMANDS = {
    'orchestrator': ['ros2', 'launch', 'orchestrator', 'orchestrator_bringup.launch.py'],
    'cyclo_data': ['ros2', 'run', 'cyclo_data', 'cyclo_data_node'],
    'bt_node': ['ros2', 'launch', 'orchestrator', 'bt_node.launch.py'],
}

_procs = {}      # name -> (Popen, started monotonic)
_stopped = {}    # name -> (exit code, stopped monotonic)


def _alive(name):
    entry = _procs.get(name)
    if entry is None:
        return False
    proc, _ = entry
    if proc.poll() is None:
        return True
    _stopped[name] = (proc.returncode, time.monotonic())
    del _procs[name]
    return False


def up(name):
    """Start a service (idempotent). Returns (ok, message)."""
    if name not in COMMANDS:
        return False, f'{name}: not available when Cyclo runs natively'
    if _alive(name):
        return True, f'{name} already up'
    cmd = list(COMMANDS[name])
    if name == 'bt_node' and BT_ROBOT_TYPE_FILE.exists():
        robot_type = BT_ROBOT_TYPE_FILE.read_text().strip()
        if robot_type:
            cmd.append(f'robot_type:={robot_type}')
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log = open(LOG_DIR / f'{name}.log', 'a')
    log.write(f'\n===== {time.strftime("%F %T")} start: {" ".join(cmd)}\n')
    log.flush()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    _procs[name] = (proc, time.monotonic())
    return True, f'{name} started (pid {proc.pid})'


def down(name, wait_s=10.0):
    """Stop a service: SIGINT to its process group (ros2 launch shuts its
    nodes down cleanly), then SIGTERM, then SIGKILL."""
    if not _alive(name):
        return True, f'{name} already down'
    proc, _ = _procs[name]
    for sig, grace in ((signal.SIGINT, wait_s), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=grace)
            break
        except subprocess.TimeoutExpired:
            continue
    _alive(name)
    return True, f'{name} stopped'


def svstat(name):
    """Status in s6-svstat's words, which the supervisor parses:
    'up (pid 1234) 37 seconds' / 'down (exitcode 1) 3 seconds'."""
    now = time.monotonic()
    if _alive(name):
        proc, started = _procs[name]
        return f'up (pid {proc.pid}) {int(now - started)} seconds'
    if name in _stopped:
        code, at = _stopped[name]
        return f'down (exitcode {code}) {int(now - at)} seconds'
    return 'down (not started yet) 0 seconds'


def stop_all():
    for name in list(_procs):
        down(name)
