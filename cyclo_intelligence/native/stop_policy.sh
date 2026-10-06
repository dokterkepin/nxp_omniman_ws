#!/usr/bin/env bash
# Emergency stop: end every policy-backend process of cyclo_intelligence/native, so nothing
# keeps publishing actions to the arm. Works without the web UI or the launch.
#
#   src/cyclo_intelligence/native/stop_policy.sh
#
# Stops the processes of the LeRobot backend (backend_main.py) - the engine,
# the runtime that publishes the actions, and the trainer. It does not touch
# ros2_control, Nav2 or anything else. The arm then stays where it is.

set -u

pids="$(pgrep -f 'cyclo_intelligence/native/backend_main.py' || true)"
if [ -z "$pids" ]; then
    echo "[stop_policy] no policy process is running"
    exit 0
fi
echo "[stop_policy] stopping:"
ps -o pid=,etime=,args= -p $(echo $pids | tr ' ' ',') | cut -c1-110
kill -KILL $pids 2>/dev/null
sleep 1
left="$(pgrep -f 'cyclo_intelligence/native/backend_main.py' || true)"
if [ -n "$left" ]; then
    echo "[stop_policy] STILL RUNNING: $left - try: sudo kill -9 $left" >&2
    exit 1
fi
echo "[stop_policy] done - no policy process is running"
