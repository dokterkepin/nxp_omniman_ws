#!/usr/bin/env python3
"""
Cyclo's supervisor API (docker/supervisor_api, unmodified) running natively.

Inside Cyclo's container the supervisor starts and stops services with s6
(`s6-rc -u|-d change <name>`, `s6-svstat /run/service/<name>`) and keeps the
bt_node robot type in /run/cyclo_intelligence/. Natively there is no s6 and
/run is root's, so before serving, this redirects exactly those to
services.py:

    _run('s6-rc', ...) / _run('s6-svstat', ...)   -> services.up/down/svstat
    os.path.isdir('/run/service[/<name>]')         -> True for known services
    _BT_ROBOT_TYPE_FILE                             -> <CYCLO_HOME>/run/...

Policy backends: the routes that pull / start / stop / report Cyclo's lerobot
container are replaced by native ones driving services.py's backend processes
(LeRobot 0.6 in the cyclo_lerobot conda env), answering in the same shape -
the UI and the tree engine's LOAD see a "running" backend with main-runtime and
engine-process "up". GR00T is not set up natively.

Everything else - every route, the BT tree store, the robot-type checks - is
Cyclo's own code. Mission Canvas's Nav2-in-a-container still needs Docker.

Services listed in CYCLO_AUTOSTART (e.g. "orchestrator,cyclo_data,bt_node")
start with the supervisor - bt_node for CYCLO_ROBOT_TYPE - as
physical_ai_server_bringup.launch.py started everything at once; the UI's
buttons still stop and start them.

Run by orchestrator's omniman_cyclo_bringup.launch.py, with PYTHONPATH containing
src/cyclo_intelligence/docker.
"""

import asyncio
import json
import os
import posixpath
import sys

from supervisor_api import app as cyclo   # noqa: E402  Cyclo's supervisor module
from fastapi.responses import StreamingResponse
import uvicorn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import services   # noqa: E402

_original_run = cyclo._run


async def _run(*cmd, **kwargs):
    """s6 commands go to services.py; anything else runs as before."""
    if cmd and cmd[0] == 's6-rc' and len(cmd) == 4 and cmd[2] == 'change':
        ok, message = (services.up if cmd[1] == '-u' else services.down)(cmd[3])
        return cyclo._S6Result(rc=0 if ok else 1, stdout=message, stderr='' if ok else message)
    if cmd and cmd[0] == 's6-svstat' and len(cmd) == 2:
        return cyclo._S6Result(rc=0, stdout=services.svstat(posixpath.basename(cmd[1])),
                               stderr='')
    return await _original_run(*cmd, **kwargs)


class _Path:
    """os.path, except the s6 service dirs the supervisor looks for exist."""

    def __getattr__(self, name):
        return getattr(os.path, name)

    @staticmethod
    def isdir(path):
        path = str(path)
        if path == '/run/service':
            return True
        if path.startswith('/run/service/'):
            return posixpath.basename(path) in services.COMMANDS
        return os.path.isdir(path)


class _Os:
    """os, with path replaced - only inside Cyclo's supervisor module."""
    path = _Path()

    def __getattr__(self, name):
        return getattr(os, name)


cyclo._run = _run
cyclo.os = _Os()
cyclo._BT_ROBOT_TYPE_FILE = str(services.BT_ROBOT_TYPE_FILE)
services.RUN_DIR.mkdir(parents=True, exist_ok=True)


# ---- policy backends: native processes instead of containers -------------------

_NATIVE = {('/backends/{name}/' + a, m) for a, m in (
    ('pull', 'POST'), ('start', 'POST'), ('restart', 'POST'), ('recreate', 'POST'),
    ('stop', 'POST'), ('status', 'GET'))}
cyclo.app.router.routes[:] = [
    r for r in cyclo.app.router.routes
    if not any((getattr(r, 'path', None), m) in _NATIVE
               for m in (getattr(r, 'methods', None) or ()))
]


def _image(name):
    if name in services.BACKENDS:
        return f'native:{services.LEROBOT_ENV}'
    return cyclo._require_known_backend(name)['image']


@cyclo.app.get('/backends/{name}/status', response_model=cyclo.BackendStatus)
async def backend_status(name: str):
    cyclo._require_known_backend(name)
    installed = services.backend_installed(name)
    state, procs = services.backend_state(name)
    return cyclo.BackendStatus(
        name=name, image=_image(name), image_pulled=installed,
        image_status='current' if installed else 'missing',
        container_state=state, raw_state=state,
        services=[cyclo.ServiceStatus(name=n, raw=raw, **cyclo._parse_svstat(raw))
                  for n, _, raw in procs])


@cyclo.app.post('/backends/{name}/pull')
async def backend_pull(name: str):
    """Nothing to download natively: the conda env is the 'image'."""
    cyclo._require_known_backend(name)

    def events():
        if services.backend_installed(name):
            yield f'event: done\ndata: {json.dumps({"image": _image(name), "ok": True})}\n\n'
        else:
            message = (f'{name} is not installed natively - run '
                       'cyclo_intelligence/native/install.sh')
            payload = json.dumps({'image': _image(name), 'message': message})
            yield f'event: error\ndata: {payload}\n\n'
    return StreamingResponse(events(), media_type='text/event-stream')


async def _act(*steps):
    ok, message = True, ''
    for step, name in steps:
        ok, message = await asyncio.to_thread(step, name)
        if not ok:
            break
    return cyclo.ActionResult(ok=ok, message=message)


@cyclo.app.post('/backends/{name}/start', response_model=cyclo.ActionResult)
async def backend_start(name: str, auto_provision: bool = False):
    cyclo._require_known_backend(name)
    return await _act((services.backend_up, name))


@cyclo.app.post('/backends/{name}/restart', response_model=cyclo.ActionResult)
async def backend_restart(name: str, auto_provision: bool = False):
    cyclo._require_known_backend(name)
    return await _act((services.backend_down, name), (services.backend_up, name))


@cyclo.app.post('/backends/{name}/recreate', response_model=cyclo.ActionResult)
async def backend_recreate(name: str):
    cyclo._require_known_backend(name)
    return await _act((services.backend_down, name), (services.backend_up, name))


@cyclo.app.post('/backends/{name}/stop', response_model=cyclo.ActionResult)
async def backend_stop(name: str):
    cyclo._require_known_backend(name)
    return await _act((services.backend_down, name))


def autostart():
    for leftover in services.kill_leftover_backends():
        print(f'[supervisor_native] stopped a leftover policy process: {leftover}', flush=True)
    names = [n.strip() for n in os.environ.get('CYCLO_AUTOSTART', '').split(',') if n.strip()]
    if 'bt_node' in names:
        robot_type = cyclo._validate_bt_robot_type(os.environ.get('CYCLO_ROBOT_TYPE', ''))
        services.BT_ROBOT_TYPE_FILE.write_text(robot_type + '\n')
    for name in names:
        # a policy backend (lerobot) is not a plain service: it is a few processes
        start = services.backend_up if name in services.BACKENDS else services.up
        ok, message = start(name)
        print(f'[supervisor_native] {message}', flush=True)


def main():
    host = os.environ.get('CYCLO_SUPERVISOR_API_HOST', '127.0.0.1')
    port = int(os.environ.get('CYCLO_SUPERVISOR_API_PORT', '7100'))
    autostart()
    try:
        uvicorn.run(cyclo.app, host=host, port=port, log_level='info')
    finally:
        services.stop_all()


if __name__ == '__main__':
    main()
