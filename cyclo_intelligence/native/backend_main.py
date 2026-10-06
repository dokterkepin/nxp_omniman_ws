#!/usr/bin/env python3
"""
Start one of Cyclo's policy-backend programs with its INFO messages shown.

    backend_main.py -m engine_process
    backend_main.py lerobot_trainer.py

Cyclo's Zenoh SDK sets its logger - and its output handler - to WARNING, so
the runtime's own progress ("EngineCommand service up", "loaded <model>")
never reaches the log. This only raises that log level, then runs the program
unchanged. Used by services.py for the native backend only.

Safety: a running policy publishes actions to the arm until told to stop, and
Cyclo has no watchdog - closing the launch, or the supervisor being killed,
left it running unseen. So this exits the moment the supervisor that started it
is gone (its parent process changes), whatever the reason - Ctrl+C, a crash,
kill -9 - and the arm gets no more commands.
"""

import logging
import os
import runpy
import sys
import threading
import time


def exit_with_parent():
    parent = os.getppid()

    def watch():
        while os.getppid() == parent:
            time.sleep(0.2)
        try:       # the output pipe went down with the supervisor: exit regardless
            print('[backend_main] the supervisor that started this is gone - '
                  'exiting, so no policy keeps driving the arm', flush=True)
        finally:
            os._exit(1)

    threading.Thread(target=watch, daemon=True).start()


exit_with_parent()

sdk = logging.getLogger('zenoh_ros2_sdk')
import zenoh_ros2_sdk.logger  # noqa: E402,F401  creates the SDK's handler
sdk.setLevel(logging.INFO)
sdk.propagate = False         # it has its own handler; LeRobot's root one printed each line twice
for handler in sdk.handlers:
    handler.setLevel(logging.INFO)

if len(sys.argv) >= 3 and sys.argv[1] == '-m':
    sys.argv = [sys.argv[2], *sys.argv[3:]]
    runpy.run_module(sys.argv[0], run_name='__main__', alter_sys=True)
else:
    sys.argv = sys.argv[1:]
    runpy.run_path(sys.argv[0], run_name='__main__')
