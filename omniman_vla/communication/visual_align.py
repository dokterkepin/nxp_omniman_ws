#!/usr/bin/env python3
"""
Moves the base until the cup sits where the pick policy expects it in the
wrist camera - the base correction step between Nav2 and the pick policy, as a
P-controller instead of a learned policy.

    ~/run     std_srvs/srv/Trigger   acquire control, start aligning
    ~/stop    std_srvs/srv/Trigger   stop, release control
    ~/status  std_msgs/msg/String    latched
              idle | searching | aligning | aligned | failed: <why>
    ~/target  std_msgs/msg/String    latched, in: which detection to align to -
              its class_id (e.g. "black square"); empty = the first detection

A run: acquire control as owner_name ("align"), then
  SEARCHING  only until the target is first seen: turn at once and without
             stopping at search_speed in search_direction (+1 left, -1 right)
             until search_turns full turns are covered (odometry), then
             fail. search_turns 0 = do not turn: wait lost_s for it, or with
             lost_s 0 wait for ever. Once seen, a run never searches again -
             see LOST below.
  ALIGNING   cup in view: one P-controller per axis, each clamped to
             [min_*, max_*] and zero inside its tolerance
               cup x (left/right)  -> angular_z  x_correction: rotate
                                   -> linear_y   x_correction: strafe
               cup y (up/down)     -> linear_x   off when k_forward is 0
             strafe can also hold the heading the cup was first seen at, with
             angular_z (k_heading > 0), so all three axes move
  ALIGNED    every axis within tolerance for settle_frames detections in a
             row - the "done" the mission waits for

LOST - detections drop out (glare, an unusual angle, motion blur). Turning
away to search and coming back made the base swing back and forth, so once
the target has been seen a run never searches again: it keeps
aligning on the last detection, and each new detection replaces it. Only new
detections count toward settle_frames. If nothing new arrives for lost_s the
run stops and fails (0 = never - it would keep driving on the old position).

A run also ends - base stopped, control given back if still held - when
  - the target does not show within search_turns turns
  - the target stays lost for lost_s
  - the base has moved max_travel_m from where the run started
  - timeout_s passes
  - control is taken away (a forced acquire)
  - ~/stop is called, or the node shuts down
max_travel_m, timeout_s and lost_s at 0 switch that limit off.

Aligns to the FIRST detection on detections_topic, whatever it is: with
sam_detector.py (started by control_launch.py) that is the first of its
`prompts` in view - "yellow cup lid", "black square", any short noun phrase.
This node itself is plain ROS - no torch.

Settings: config/visual_align.yaml, section visual_align - read from the
file directly (no ROS parameters). Saved edits apply within a second; topics
and rate_hz only at start.

Run (with control_arbiter):
  ros2 launch omniman_vla control_launch.py
  ros2 service call /visual_align/run std_srvs/srv/Trigger
  ros2 topic echo /visual_align/status
"""

import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

CALL_TIMEOUT_S = 15.0

BUSY = ('searching', 'aligning')

# Settings visual_align needs in its section of the config file.
REQUIRED = [
    'owner_name', 'detections_topic', 'cmd_vel_topic', 'odom_topic', 'rate_hz',
    'x_correction', 'aim_x', 'tolerance_x', 'aim_y', 'tolerance_y', 'settle_frames',
    'min_score', 'k_angular', 'max_angular', 'min_angular', 'k_lateral', 'max_lateral',
    'k_forward', 'max_forward', 'min_linear', 'k_heading', 'tolerance_heading_deg',
    'max_travel_m', 'detection_timeout_s', 'lost_s', 'search_speed', 'search_turns',
    'search_direction', 'timeout_s',
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


CONFIG_FILE = os.path.join(get_package_share_directory('omniman_vla'), 'config',
                           'visual_align.yaml')


class Config:
    """This node's section of config/visual_align.yaml - read straight from
    the file, not through ROS parameters. Re-read when the file changes
    (checked at most once a second), so saved edits apply without a restart;
    an edit that breaks the file is logged and the previous values are kept."""

    def __init__(self, section, missing, logger):
        # missing(values) -> names of the settings the section lacks.
        self.section, self.missing, self.logger = section, missing, logger
        self.checked = 0.0
        self.load()

    def load(self):
        mtime = os.stat(CONFIG_FILE).st_mtime
        with open(CONFIG_FILE) as f:
            values = (yaml.safe_load(f) or {}).get(self.section) or {}
        missing = self.missing(values)
        if missing:
            raise RuntimeError(f'{CONFIG_FILE} [{self.section}] is missing: '
                               f'{", ".join(missing)}')
        self.values, self.mtime = values, mtime

    def __getitem__(self, key):
        now = time.monotonic()
        if now - self.checked > 1.0:
            self.checked = now
            try:
                if os.stat(CONFIG_FILE).st_mtime != self.mtime:
                    self.load()
                    self.logger.info(f'reloaded {CONFIG_FILE}')
            except (OSError, yaml.YAMLError, RuntimeError, AttributeError) as e:
                self.logger.error(f'bad edit, keeping previous values: {e}')
                self.mtime = os.stat(CONFIG_FILE).st_mtime
        return self.values[key]


class VisualAlign(Node):

    def __init__(self):
        super().__init__('visual_align')
        # Every value comes from config/visual_align.yaml [visual_align].
        self.cfg = Config('visual_align', lambda v: [k for k in REQUIRED if k not in v],
                          self.get_logger())

        # Same threading as policy_runner.py: service handlers wait on other
        # services, so reentrant + multithreaded, and the lock is never held
        # across a service call.
        group = ReentrantCallbackGroup()
        self.group = group
        self.lock = threading.Lock()

        self.acquire_client = self.create_client(
            AcquireControl, '/control/acquire', callback_group=group)
        self.release_client = self.create_client(
            ReleaseControl, '/control/release', callback_group=group)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.cmd_pub = self.create_publisher(
            Twist, self.p('cmd_vel_topic'), 10)

        self.owner = ''
        self.holding = False
        self.state = ''
        self.det_sub = None
        self.started_at = 0.0

        # Latest detection, image px: when it arrived, and whether the
        # controller has used it yet (settle_frames counts detections).
        self.cup_x = None
        self.cup_y = None
        self.cup_seen_at = 0.0
        self.cup_new = False
        self.settled = 0
        self.lost_noted = False

        self.have_odom = False
        self.yaw = 0.0
        self.pos = (0.0, 0.0)
        self.start_pos = (0.0, 0.0)
        # Heading strafe holds: where the cup was first seen this run.
        self.hold_yaw = 0.0
        self.hold_set = False
        self.search_since = 0.0
        # Search progress: how far the base has turned, from odometry yaw.
        self.search_turned = 0.0
        self.search_prev_yaw = 0.0

        self.create_subscription(
            ControlOwner, '/control/owner', self.on_owner, latched, callback_group=group)
        # Set by a mission before ~/run (the place step aligns to the mark while
        # the held cup is also in view). Empty: the first detection, whatever.
        self.target = ''
        self.create_subscription(
            String, '~/target', self.on_target, latched, callback_group=group)
        self.create_subscription(
            Odometry, self.p('odom_topic'), self.on_odom, 10,
            callback_group=group)
        self.create_service(Trigger, '~/run', self.on_run, callback_group=group)
        self.create_service(Trigger, '~/stop', self.on_stop, callback_group=group)
        self.create_timer(1.0 / float(self.p('rate_hz')), self.tick,
                          callback_group=group)

        self.set_state('idle')
        self.get_logger().info(f'ready - aligns as owner "{self.owner_name()}"')

    # ---- helpers ------------------------------------------------------------

    def p(self, name):
        return self.cfg[name]

    def owner_name(self):
        return self.p('owner_name')

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.status_pub.publish(String(data=state))
            self.get_logger().info(f'-> {state}')

    def call(self, client, request):
        """Call a service and wait for its answer; None on timeout."""
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(CALL_TIMEOUT_S) else None

    def drive(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.linear.y = float(linear_y)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def axis(self, error, tolerance, k, lo, hi):
        """P-control one axis: 0 inside tolerance, else k*error with its size
        clamped to [lo, hi] - below lo the wheels do not move the base."""
        if k <= 0.0 or abs(error) <= tolerance:
            return 0.0
        return math.copysign(min(max(abs(k * error), lo), hi), error)

    def start_search(self):
        self.search_since = time.monotonic()
        self.search_turned = 0.0
        self.search_prev_yaw = self.yaw
        self.set_state('searching')

    def end_run(self, result, release=True):
        """Every way a run ends goes through here, exactly once per run."""
        with self.lock:
            if self.state not in BUSY:
                return
            self.set_state('ending')
            holding, self.holding = self.holding, False
            sub, self.det_sub = self.det_sub, None
        for _ in range(3):
            self.drive()
        if sub is not None:
            self.destroy_subscription(sub)
        if release and holding:
            req = ReleaseControl.Request()
            req.owner = self.owner_name()
            self.call(self.release_client, req)
        self.get_logger().info(f'run ended after {time.monotonic() - self.started_at:.1f}s')
        self.set_state(result)

    # ---- services -----------------------------------------------------------

    def on_run(self, request, response):
        with self.lock:
            if self.state in BUSY + ('starting', 'ending'):
                response.success = False
                response.message = f'already {self.state}'
                return response
            if not self.have_odom:
                response.success = False
                response.message = f'no odometry on {self.p("odom_topic")}'
                return response
            # Reserve the node, so a second run is refused while this one
            # starts. Not BUSY yet: the control loop leaves it alone.
            self.set_state('starting')

        acquire = AcquireControl.Request()
        acquire.owner = self.owner_name()
        acquire.node = self.get_fully_qualified_name()
        got = self.call(self.acquire_client, acquire)
        if got is None or not got.success:
            self.set_state('idle')
            response.success = False
            response.message = got.message if got else 'control_arbiter not answering'
            return response
        # Only watch for takeovers once /control/owner shows us as owner.
        deadline = time.monotonic() + 3.0
        while self.owner != self.owner_name() and time.monotonic() < deadline:
            time.sleep(0.02)

        with self.lock:
            self.holding = True
            self.started_at = time.monotonic()
            self.cup_x = None
            self.cup_new = False
            self.settled = 0
            self.lost_noted = False
            self.start_pos = self.pos
            self.hold_set = False
            self.det_sub = self.create_subscription(
                Detection2DArray, self.p('detections_topic'), self.on_detections, 1,
                callback_group=self.group)
            self.start_search()

        response.success = True
        response.message = 'aligning'
        return response

    def on_stop(self, request, response):
        if self.state in BUSY:
            self.end_run('failed: stopped')
            response.message = 'stopped'
        else:
            response.message = 'nothing running'
        response.success = True
        return response

    # ---- inputs -------------------------------------------------------------

    def on_owner(self, msg):
        self.owner = msg.owner
        if self.holding and msg.owner != self.owner_name():
            taker = msg.owner or 'a force release'
            self.end_run(f'failed: control taken by {taker}', release=False)

    def on_odom(self, msg):
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.have_odom = True

    def on_target(self, msg):
        self.target = msg.data
        self.get_logger().info(f'target: {self.target or "first detection"}')

    def on_detections(self, msg):
        best = next((d for d in msg.detections
                     if d.results and d.results[0].hypothesis.score >= self.p('min_score')
                     and (not self.target or d.results[0].hypothesis.class_id == self.target)),
                    None)
        if best is None:
            return
        with self.lock:
            self.cup_x = best.bbox.center.position.x
            self.cup_y = best.bbox.center.position.y
            self.cup_seen_at = time.monotonic()
            self.cup_new = True

    # ---- control loop -------------------------------------------------------

    def tick(self):
        if self.state not in BUSY:
            return
        now = time.monotonic()
        if 0.0 < self.p('timeout_s') < now - self.started_at:
            self.end_run(f'failed: timeout after {self.p("timeout_s"):.0f}s')
            return

        travelled = math.dist(self.pos, self.start_pos)
        if 0.0 < self.p('max_travel_m') < travelled:
            self.end_run(f'failed: moved {travelled:.2f} m, more than max_travel_m')
            return

        with self.lock:
            if self.state not in BUSY:
                return
            fresh = (self.cup_x is not None
                     and now - self.cup_seen_at < self.p('detection_timeout_s'))
            new, self.cup_new = self.cup_new, False
            result, cmd = None, (0.0, 0.0, 0.0)

            if self.state == 'searching':
                if fresh:
                    self.settled = 0
                    if not self.hold_set:
                        self.hold_yaw = self.yaw
                        self.hold_set = True
                    self.set_state('aligning')
                else:
                    result, cmd = self.search_command(now)

            if self.state == 'aligning':
                age = now - self.cup_seen_at
                if fresh:
                    if self.lost_noted:
                        self.get_logger().info(f'   target back after {age:.1f}s')
                        self.lost_noted = False
                    cmd = self.align_command(self.cup_x, self.cup_y)
                    if cmd == (0.0, 0.0, 0.0):
                        if new:
                            self.settled += 1
                        if self.settled >= self.p('settle_frames'):
                            result = 'aligned'
                            self.get_logger().info(
                                f'   target at x={self.cup_x:.0f} y={self.cup_y:.0f}, '
                                f'base moved {travelled:.2f} m')
                    else:
                        self.settled = 0
                else:
                    # Keep aligning on the last detection until a new one.
                    self.settled = 0
                    if not self.lost_noted:
                        self.get_logger().warn('   target lost - using last detection')
                        self.lost_noted = True
                    cmd = self.align_command(self.cup_x, self.cup_y)
                    if 0.0 < self.p('lost_s') < age:
                        result = f'failed: target lost for {age:.0f}s'

        if result is not None:
            self.end_run(result)
        else:
            self.drive(*cmd)

    def search_command(self, now):
        """(result, cmd) while the target has not been seen: turn without
        stopping until search_turns full turns are covered."""
        waited = now - self.search_since
        turns = float(self.p('search_turns'))
        if turns <= 0.0:
            # No turning: give the target lost_s to show up, or wait for ever.
            lost_s = self.p('lost_s')
            if 0.0 < lost_s < waited:
                return 'failed: target not in view', (0.0, 0.0, 0.0)
            return None, (0.0, 0.0, 0.0)
        # Angle covered so far, from odometry (unwrapped, either direction).
        self.search_turned += abs(wrap(self.yaw - self.search_prev_yaw))
        self.search_prev_yaw = self.yaw
        if self.search_turned < turns * 2.0 * math.pi:
            side = 1.0 if self.p('search_direction') >= 0 else -1.0
            return None, (0.0, 0.0, side * abs(self.p('search_speed')))
        return (f'failed: target not found after turning '
                f'{math.degrees(self.search_turned):.0f} deg', (0.0, 0.0, 0.0))

    def align_command(self, x, y):
        """(linear_x, linear_y, angular_z) for the target at image (x, y)."""
        ex = self.p('aim_x') - x             # + : cup left of aim
        vx = vy = wz = 0.0
        if self.p('k_forward') > 0.0:
            ey = self.p('aim_y') - y         # + : cup above aim, i.e. too far
            vx = self.axis(ey, self.p('tolerance_y'), self.p('k_forward'),
                           self.p('min_linear'), self.p('max_forward'))
        if self.p('x_correction') == 'strafe':
            vy = self.axis(ex, self.p('tolerance_x'), self.p('k_lateral'),
                           self.p('min_linear'), self.p('max_lateral'))
            wz = self.axis(wrap(self.hold_yaw - self.yaw),
                           math.radians(self.p('tolerance_heading_deg')),
                           self.p('k_heading'), self.p('min_angular'), self.p('max_angular'))
        else:
            wz = self.axis(ex, self.p('tolerance_x'), self.p('k_angular'),
                           self.p('min_angular'), self.p('max_angular'))
        return (vx, vy, wz)


def main():
    # rclpy's SIGINT handler would shut the context down before the stop and
    # release below could be sent; Python's default just raises KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VisualAlign()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        while spinner.is_alive():
            spinner.join(timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.end_run('failed: shutting down')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
