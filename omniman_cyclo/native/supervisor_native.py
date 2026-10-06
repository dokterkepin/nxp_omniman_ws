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

Everything else - every route, the BT tree store, the robot-type checks - is
Cyclo's own code. What only works in Docker (starting the GR00T / LeRobot
policy containers, Mission Canvas's Nav2-in-a-container) keeps needing Docker.

Services listed in CYCLO_AUTOSTART (e.g. "orchestrator,cyclo_data,bt_node")
start with the supervisor - bt_node for CYCLO_ROBOT_TYPE - as
physical_ai_server_bringup.launch.py started everything at once; the UI's
buttons still stop and start them.

Run by omniman_cyclo's cyclo.launch.py, with PYTHONPATH containing
src/cyclo_intelligence/docker.
"""

import os
import posixpath
import sys

from supervisor_api import app as cyclo   # noqa: E402  Cyclo's supervisor module
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


def autostart():
    names = [n.strip() for n in os.environ.get('CYCLO_AUTOSTART', '').split(',') if n.strip()]
    if 'bt_node' in names:
        robot_type = cyclo._validate_bt_robot_type(os.environ.get('CYCLO_ROBOT_TYPE', ''))
        services.BT_ROBOT_TYPE_FILE.write_text(robot_type + '\n')
    for name in names:
        ok, message = services.up(name)
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
