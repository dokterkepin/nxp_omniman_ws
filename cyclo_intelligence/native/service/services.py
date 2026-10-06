#!/usr/bin/env python3
"""
Cyclo's services as plain processes - what s6 does inside Cyclo's container,
for running Cyclo natively (orchestrator's omniman_cyclo_bringup.launch.py).

    orchestrator   ros2 launch orchestrator orchestrator_bringup.launch.py
                   (orchestrator, rosbridge, rosbag recorder, web_video_server)
    cyclo_data     ros2 run cyclo_data cyclo_data_node
    bt_node        ros2 launch orchestrator bt_node.launch.py robot_type:=<type>

Started and stopped by the UI's buttons through the supervisor
(supervisor_native.py), exactly as in the container. Each runs in its own
process group with the supervisor's environment. Its output goes to the
launch terminal, each line tagged [<name>] (as physical_ai_server_bringup
shows its nodes), and to <CYCLO_HOME>/log/services/<name>.log.

Policy backends - what Cyclo runs in its lerobot / groot containers - are
processes too, in the cyclo_lerobot conda env (CYCLO_LEROBOT_ENV):

    lerobot   engine-process   python -m engine_process   loads the policy, reads
                                                         the robot (LeRobot 0.6)
              main-runtime     python -m main_runtime     /lerobot/inference_command,
                                                         publishes the actions
              trainer          policy/lerobot/lerobot_trainer.py  /lerobot/train

They talk to ROS over Zenoh (zenoh_ros2_sdk, no ROS install), so ROS's
Python and library paths are kept out of their environment.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import yaml

# Cyclo's runtime files, outside git: nxp_omniman_ws/cyclo
# (set by omniman_cyclo_bringup.launch.py).
CYCLO_HOME = Path(os.environ.get('CYCLO_HOME', Path(__file__).resolve().parents[4] / 'cyclo'))
RUN_DIR = CYCLO_HOME / 'run'
LOG_DIR = CYCLO_HOME / 'log' / 'services'
BT_ROBOT_TYPE_FILE = RUN_DIR / 'bt_node_robot_type'

COMMANDS = {
    'orchestrator': ['ros2', 'launch', 'orchestrator', 'orchestrator_bringup.launch.py'],
    'cyclo_data': ['ros2', 'run', 'cyclo_data', 'cyclo_data_node'],
    'bt_node': ['ros2', 'launch', 'orchestrator', 'bt_node.launch.py'],
}

CYCLO_DIR = Path(os.environ.get('CYCLO_DIR', Path(__file__).resolve().parents[2]))
LEROBOT_ENV = Path(os.environ.get('CYCLO_LEROBOT_ENV',
                                  Path.home() / 'miniconda3' / 'envs' / 'cyclo_lerobot'))

_TRAINER = CYCLO_DIR / 'cyclo_brain' / 'policy' / 'lerobot' / 'lerobot_trainer.py'
POLICY_DIR = Path(__file__).resolve().parents[1] / 'policy'
BACKEND_CONFIG = POLICY_DIR / 'lerobot_backend.yaml'
ENGINES = ('lerobot_engine', 'omniman_act')

# Backend processes, in start order: main-runtime waits for engine-process.
_MAIN = str(POLICY_DIR / 'backend_main.py')      # shows the runtime's INFO messages
BACKENDS = {
    'lerobot': {
        'engine-process': [_MAIN, '-m', 'engine_process'],
        'main-runtime': [_MAIN, '-m', 'main_runtime'],
        'trainer': [_MAIN, str(_TRAINER)],
    },
}
# What Cyclo's UI and tree engine check for "backend up" (container services).
BACKEND_REPORTED = ['main-runtime', 'engine-process']

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
    _pid_file(name).unlink(missing_ok=True)
    return False


_print_lock = threading.Lock()


def _pump(name, stream, log):
    """Each output line to the log file and, tagged, to the terminal."""
    tag = f'[{name}] '
    for raw in iter(stream.readline, b''):
        line = raw.decode(errors='replace')
        log.write(line)
        log.flush()
        with _print_lock:
            sys.stdout.write(tag + line)
            sys.stdout.flush()
    log.close()


PID_DIR = RUN_DIR / 'backend'      # one pid file per running policy-backend process


def _pid_file(name):
    return PID_DIR / f'{name.replace("/", "_")}.pid'


def _spawn(name, cmd, env=None):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log = open(LOG_DIR / f'{name.replace("/", "_")}.log', 'a')
    log.write(f'\n===== {time.strftime("%F %T")} start: {" ".join(cmd)}\n')
    log.flush()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    threading.Thread(target=_pump, args=(name, proc.stdout, log), daemon=True).start()
    _procs[name] = (proc, time.monotonic())
    if name.split('/')[0] in BACKENDS:
        PID_DIR.mkdir(parents=True, exist_ok=True)
        _pid_file(name).write_text(f'{proc.pid}\n')
    return proc


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
    proc = _spawn(name, cmd)
    return True, f'{name} started (pid {proc.pid})'


# ---- policy backends ----------------------------------------------------------

def backend_installed(backend):
    """The env has its packages (not just the folder: it exists mid-install)."""
    site = LEROBOT_ENV / 'lib' / 'python3.12' / 'site-packages'
    return backend in BACKENDS and all(
        (site / pkg).is_dir() for pkg in ('lerobot', 'torch', 'zenoh', 'zenoh_ros2_sdk'))


def backend_engine():
    """The engine chosen in lerobot_backend.yaml (read at every backend start)."""
    with open(BACKEND_CONFIG) as f:
        engine = (yaml.safe_load(f) or {}).get('engine', 'lerobot_engine')
    if engine not in ENGINES:
        raise ValueError(f'{BACKEND_CONFIG}: engine "{engine}" - one of {", ".join(ENGINES)}')
    return engine


def _backend_env(backend):
    """Like the container's: Cyclo's runtime, SDKs and engine on PYTHONPATH,
    nothing of ROS (these processes speak Zenoh themselves)."""
    brain = CYCLO_DIR / 'cyclo_brain'
    drop = ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'CMAKE_PREFIX_PATH', 'PYTHONHOME', 'VIRTUAL_ENV')
    env = {k: v for k, v in os.environ.items() if k not in drop}
    engine = backend_engine()
    paths = [brain / 'policy' / backend, brain / 'policy' / 'common' / 'runtime',
             brain / 'sdk' / 'robot_client', brain / 'sdk' / 'action_chunk_processing']
    env.update({
        'PYTHONPATH': os.pathsep.join(str(p) for p in paths),
        'PATH': f'{LEROBOT_ENV / "bin"}{os.pathsep}{env.get("PATH", "")}',
        'POLICY_BACKEND': backend,
        'POLICY_ENGINE_MODULE': engine,
        'ROBOT_CLIENT_SDK_PATH': str(brain / 'sdk' / 'robot_client'),
        'ACTION_CHUNK_PROCESSING_SDK_PATH': str(brain / 'sdk' / 'action_chunk_processing'),
        'ZENOH_SDK_PATH': '',            # pip-installed in the env
        'ZENOH_ROUTER_IP': env.get('ZENOH_ROUTER_IP', '127.0.0.1'),
        'ZENOH_ROUTER_PORT': env.get('ZENOH_ROUTER_PORT', '7447'),
        'ORCHESTRATOR_CONFIG_PATH': env.get(
            'ORCHESTRATOR_CONFIG_PATH', str(CYCLO_DIR / 'shared' / 'shared' / 'robot_configs')),
        'PYTHONUNBUFFERED': '1',
    })
    if engine == 'omniman_act':
        env['POSTPROCESS_ACTIONS'] = 'false'      # one action per step, no 100 Hz interpolation
    return env


def backend_up(backend):
    """Start a backend's processes (idempotent). Returns (ok, message)."""
    if backend not in BACKENDS:
        return False, f'{backend}: not set up to run natively (it needs Docker)'
    if not backend_installed(backend):
        return False, (f'{backend}: no conda env at {LEROBOT_ENV} - run '
                       'cyclo_intelligence/native/install.sh')
    try:
        env = _backend_env(backend)
    except (OSError, ValueError) as e:
        return False, f'{backend}: {e}'
    print(f'[services] {backend} engine: {env["POLICY_ENGINE_MODULE"]}', flush=True)
    started = []
    for proc_name, args in BACKENDS[backend].items():
        name = f'{backend}/{proc_name}'
        if not _alive(name):
            _spawn(name, [str(LEROBOT_ENV / 'bin' / 'python'), *args], env=env)
            started.append(proc_name)
    return True, f'{backend}: ' + (f'started {", ".join(started)}' if started else 'already up')


def backend_down(backend):
    for proc_name in reversed(list(BACKENDS.get(backend, {}))):
        down(f'{backend}/{proc_name}')
    return True, f'{backend} stopped'


def backend_state(backend):
    """(container_state, [(service, state, svstat text)]) in Cyclo's words."""
    names = [f'{backend}/{p}' for p in BACKENDS.get(backend, {})]
    alive = [n for n in names if _alive(n)]
    if alive and len(alive) == len(names):
        state = 'running'
    elif alive or any(n in _stopped for n in names):
        state = 'exited'
    else:
        state = 'not_created'
    services = []
    for proc_name in BACKEND_REPORTED:
        name = f'{backend}/{proc_name}'
        services.append((proc_name, 'up' if _alive(name) else 'down', svstat(name)))
    return state, services


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
    """Everything, the policy first: it is what moves the arm, and the slow
    ROS launches behind it must not keep it waiting - the launch gives the
    supervisor only a few seconds before killing it."""
    names = list(_procs)
    for name in sorted(names, key=lambda n: n.split('/')[0] not in BACKENDS):
        down(name, wait_s=2.0 if name.split('/')[0] in BACKENDS else 10.0)


def kill_leftover_backends():
    """Stop policy-backend processes a previous supervisor left running (it was
    killed before it could stop them). Found by their pid files; only a
    process still running our backend_main.py is touched."""
    found = []
    for path in sorted(PID_DIR.glob('*.pid')) if PID_DIR.is_dir() else []:
        try:
            pid = int(path.read_text().strip())
            cmdline = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        except (OSError, ValueError):
            path.unlink(missing_ok=True)
            continue
        if 'backend_main.py' in cmdline:
            try:
                os.killpg(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                os.kill(pid, signal.SIGKILL)
            found.append(f'{path.stem} (pid {pid})')
        path.unlink(missing_ok=True)
    return found
