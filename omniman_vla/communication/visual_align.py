#!/usr/bin/env python3
"""
Moves the base until the cup sits where the pick policy expects it in the
wrist camera - the base correction step between Nav2 and the pick policy, as a
P-controller instead of a learned policy.

    ~/run     std_srvs/srv/Trigger   acquire control, start aligning
    ~/stop    std_srvs/srv/Trigger   stop, release control
    ~/status  std_msgs/msg/String    latched
              idle | searching | aligning | aligned | failed: <why>

A run: acquire control as owner_name ("align"), then
  SEARCHING  no cup in view: turn at search_speed for up to search_time_s,
             toward the side the cup was last seen on (search_direction if
             it was never seen), until the cup shows. search_time_s 0 = do
             not turn: wait lost_s for it, or with lost_s 0 wait for ever
  ALIGNING   cup in view: one P-controller per axis, each clamped to
             [min_*, max_*] and zero inside its tolerance
               cup x (left/right)  -> angular_z  x_correction: rotate
                                   -> linear_y   x_correction: strafe
               cup y (up/down)     -> linear_x   off when k_forward is 0
             strafe can also hold the heading the cup was first seen at, with
             angular_z (k_heading > 0), so all three axes move
  ALIGNED    every axis within tolerance for settle_frames detections in a
             row - the "done" the mission waits for
then stop the base and release control. aligned / failed stays on ~/status
until the next run.

One point in the image gives two errors, the base has three axes: x is fixed
either by turning or by sliding sideways, never both. rotate is what the demos
did; strafe keeps the heading Nav2 arrived with.

The numbers come from the base_correct demos (omniman_base_correct_v6, cup
found by cup_detector.py): with the cup left of x=280 the operator turned left
(+angular_z), right of 360 turned right, and in between stopped 95% of the
time. After correcting, the cup sat at x 299-343 and y 201-260 (middle half of
the episodes), so aim 320, 245.

Signs, with the arm at home and the camera looking forward: cup left of aim ->
turn left (+angular_z) or slide left (+linear_y); cup higher than aim (further
away) -> forward (+linear_x).

A run also ends - base stopped, control given back if still held - when
  - the cup does not show within search_time_s of turning
  - the base has moved max_travel_m from where the run started
  - timeout_s passes
  - control is taken away (a forced acquire)
  - ~/stop is called, or the node shuts down
max_travel_m, timeout_s and lost_s at 0 switch that limit off.

Needs cup_detector.py running (GPU PC, lerobot_jazzy env). This node itself is
plain ROS - no torch.

Run (with control_arbiter):
  ros2 launch omniman_vla control_launch.py
  ros2 service call /visual_align/run std_srvs/srv/Trigger
  ros2 topic echo /visual_align/status
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

CALL_TIMEOUT_S = 15.0

# Extra wait for the first detection of a run: the detector only starts once
# it discovers this node's subscription, which is made when the run starts.
# Turning before then could turn a cup that is in view out of it.
FIRST_DETECTION_S = 2.0

BUSY = ('searching', 'aligning')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def declare_from_yaml(node, names):
    """Declare parameters with no default: every value must come from the
    params file. Any number type is accepted (300 or 300.0). Raises, naming
    the missing ones, if the file does not set them all."""
    for name in names:
        node.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))
    missing = [n for n in names if node.get_parameter(n).type_ == Parameter.Type.NOT_SET]
    if missing:
        raise RuntimeError(
            f'{node.get_name()}: not set in the params file: {", ".join(missing)} '
            '- run it with --params-file config/visual_align.yaml')


class VisualAlign(Node):

    def __init__(self):
        super().__init__('visual_align')
        # All values come from config/visual_align.yaml - none are set here.
        declare_from_yaml(self, [
            'owner_name', 'detections_topic', 'cmd_vel_topic', 'odom_topic', 'rate_hz',
            'x_correction', 'aim_x', 'tolerance_x', 'aim_y', 'tolerance_y', 'settle_frames',
            'min_score', 'k_angular', 'max_angular', 'min_angular', 'k_lateral',
            'max_lateral', 'k_forward', 'max_forward', 'min_linear', 'k_heading',
            'tolerance_heading_deg', 'max_travel_m', 'detection_timeout_s', 'lost_s',
            'search_speed', 'search_time_s', 'search_direction', 'timeout_s',
        ])

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
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.owner = ''
        self.holding = False
        self.state = ''
        self.det_sub = None
        self.started_at = 0.0

        # Latest detection: cup x, when it arrived, and whether the controller
        # has used it yet (settle_frames counts detections, not ticks).
        self.cup_x = None
        self.cup_y = None
        self.cup_seen_at = 0.0
        self.cup_new = False
        self.last_side = 0          # +1 cup was left of aim, -1 right, 0 never seen
        self.settled = 0

        self.have_odom = False
        self.yaw = 0.0
        self.pos = (0.0, 0.0)
        self.start_pos = (0.0, 0.0)
        # Heading strafe holds: where the cup was first seen this run.
        self.hold_yaw = 0.0
        self.hold_set = False
        self.search_since = 0.0
        self.search_side = 1

        self.create_subscription(
            ControlOwner, '/control/owner', self.on_owner, latched, callback_group=group)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10,
            callback_group=group)
        self.create_service(Trigger, '~/run', self.on_run, callback_group=group)
        self.create_service(Trigger, '~/stop', self.on_stop, callback_group=group)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick,
                          callback_group=group)

        self.set_state('idle')
        self.get_logger().info(f'ready - aligns as owner "{self.owner_name()}"')

    # ---- helpers ------------------------------------------------------------

    def p(self, name):
        return self.get_parameter(name).value

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
        """Turn toward the side the cup was last seen on, else search_direction."""
        self.search_side = self.last_side or (1 if self.p('search_direction') >= 0 else -1)
        self.search_since = time.monotonic()
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
            self.last_side = 0
            self.settled = 0
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

    def on_detections(self, msg):
        best = next((d for d in msg.detections
                     if d.results and d.results[0].hypothesis.score >= self.p('min_score')),
                    None)
        if best is None:
            return
        with self.lock:
            self.cup_x = best.bbox.center.position.x
            self.cup_y = best.bbox.center.position.y
            self.cup_seen_at = time.monotonic()
            self.cup_new = True
            self.last_side = 1 if self.cup_x < self.p('aim_x') else -1

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
                if fresh:
                    cmd = self.align_command()
                    if cmd == (0.0, 0.0, 0.0):
                        if new:
                            self.settled += 1
                        if self.settled >= self.p('settle_frames'):
                            result = 'aligned'
                            self.get_logger().info(
                                f'   cup at x={self.cup_x:.0f} y={self.cup_y:.0f}, '
                                f'base moved {travelled:.2f} m')
                    else:
                        self.settled = 0
                elif now - self.cup_seen_at > self.p('lost_s'):
                    self.get_logger().warn('   cup lost - searching again')
                    self.start_search()

        if result is not None:
            self.end_run(result)
        else:
            self.drive(*cmd)

    def search_command(self, now):
        """(result, cmd) while the cup is not in view."""
        waited = now - self.search_since
        first = FIRST_DETECTION_S if self.cup_x is None else 0.0
        turn_s = self.p('search_time_s')
        if turn_s <= 0.0:
            # No turning: give the cup lost_s to show up, or wait for ever.
            lost_s = self.p('lost_s')
            if 0.0 < lost_s < waited - first:
                return 'failed: cup not in view', (0.0, 0.0, 0.0)
            return None, (0.0, 0.0, 0.0)
        if waited < first:
            return None, (0.0, 0.0, 0.0)
        if waited - first < turn_s:
            return None, (0.0, 0.0, self.search_side * self.p('search_speed'))
        return f'failed: cup not found after turning {turn_s:.0f}s', (0.0, 0.0, 0.0)

    def align_command(self):
        """(linear_x, linear_y, angular_z) for the latest cup position."""
        ex = self.p('aim_x') - self.cup_x    # + : cup left of aim
        ey = self.p('aim_y') - self.cup_y    # + : cup above aim, i.e. too far
        vx = vy = wz = 0.0
        if self.p('k_forward') > 0.0:
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
