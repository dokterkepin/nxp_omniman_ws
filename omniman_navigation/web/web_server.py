#!/usr/bin/env python3
"""
Static file server for the touch UI (web/), plus a small places API.

    GET    /api/places          -> {"name": {"x": .., "y": .., "yaw": ..}, ...}
    POST   /api/places          <- {"name": .., "x": .., "y": .., "yaw": ..}
    DELETE /api/places/<name>

Places live in config/poses.yaml, the same file pick_place_mission.py and
pick_place_shuttle.py read - so saving pick_area from the iPad moves where the
missions drive. Map frame, yaw in degrees.

The file is rewritten whole. It is written next to its real path and renamed
over it, so a crash mid-write cannot leave half a file, and a symlinked
install keeps pointing at the updated source copy.

Lives in web/ beside the page; started by web_ui_launch.py. Not a ROS node.
"""

import argparse
import functools
import http.server
import json
import math
import os
import re
import tempfile
import threading

import yaml

NAME = re.compile(r'^[A-Za-z0-9_-]{1,40}$')

HEADER = """# Named places in the map frame, yaw in DEGREES.
#
# Written by the web UI ("Save here" stores where the robot really is), so it
# is rewritten as a whole: hand edits are fine, extra comments are not kept.
#
#   home        - where pick_place_mission.py / pick_place_shuttle.py start and end
#   pick_area   - where the missions drive to pick
#   place_area  - where the missions drive to place
# Any other name is just a saved place to drive to from the web UI.
"""


class Places:
    """poses.yaml, read and written under one lock."""

    def __init__(self, path):
        self.path = os.path.realpath(path)
        self.lock = threading.Lock()

    def read(self):
        try:
            with open(self.path) as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}

    def _write(self, places):
        lines = [HEADER]
        for name, p in places.items():
            lines.append(f"{name}: {{x: {float(p['x']):.3f}, y: {float(p['y']):.3f}, "
                         f"yaw: {float(p['yaw']):.1f}}}\n")
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path), suffix='.tmp')
        with os.fdopen(fd, 'w') as f:
            f.writelines(lines)
        os.replace(tmp, self.path)

    def save(self, name, x, y, yaw):
        with self.lock:
            places = self.read()
            places[name] = {'x': x, 'y': y, 'yaw': yaw}
            self._write(places)
            return places

    def delete(self, name):
        with self.lock:
            places = self.read()
            places.pop(name, None)
            self._write(places)
            return places


class Handler(http.server.SimpleHTTPRequestHandler):

    def __init__(self, *args, places, **kwargs):
        self.places = places
        super().__init__(*args, **kwargs)

    def _json(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/api/places':
            self._json(200, self.places.read())
        elif path.endswith('.py'):
            # This script lives in the folder it serves; do not hand out code.
            self._json(404, {'error': 'not found'})
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != '/api/places':
            return self._json(404, {'error': 'not found'})
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            name = str(body['name'])
            x, y, yaw = (float(body[k]) for k in ('x', 'y', 'yaw'))
        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
            return self._json(400, {'error': 'expected {name, x, y, yaw}'})
        if not NAME.match(name):
            return self._json(400, {'error': 'name: letters, digits, _ or -, up to 40'})
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return self._json(400, {'error': 'x, y, yaw must be numbers'})
        self._json(200, self.places.save(name, x, y, yaw))

    def do_DELETE(self):
        prefix = '/api/places/'
        name = self.path[len(prefix):] if self.path.startswith(prefix) else ''
        if not NAME.match(name):
            return self._json(404, {'error': 'not found'})
        self._json(200, self.places.delete(name))

    def log_message(self, fmt, *args):
        # Keep the launch output readable: only the API calls, not every file.
        # Filter on the path - on errors args[0] is a status code, not a string.
        if getattr(self, 'path', '').startswith('/api/'):
            super().log_message(fmt, *args)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--port', type=int, default=8081)
    ap.add_argument('--web-dir', required=True)
    ap.add_argument('--poses', required=True, help='path to poses.yaml')
    args, _ = ap.parse_known_args()     # tolerate --ros-args from launch

    handler = functools.partial(Handler, places=Places(args.poses),
                                directory=args.web_dir)
    server = http.server.ThreadingHTTPServer(('0.0.0.0', args.port), handler)
    print(f'serving {args.web_dir} on :{args.port}, places in {args.poses}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
