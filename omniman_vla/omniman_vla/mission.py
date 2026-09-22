"""
Building blocks for missions as behaviour trees (py_trees).

A mission only draws its tree; this module does the rest:

    import py_trees
    from omniman_vla.mission import Navigate, Align, PolicyStep, Holding, run_mission

    def build(robot):
        return py_trees.composites.Sequence('fetch', memory=True, children=[
            Navigate(robot, 'shelf'),
            Align(robot, 'green bottle'),
            PolicyStep(robot, 'pick', 'pick the bottle'),
            Holding(robot, 'grasp succeeded', holding=True),
        ])

    if __name__ == '__main__':
        run_mission(build, node_name='fetch_mission')

The steps
    Navigate(robot, place)                 drive with Nav2 to a place in poses.yaml
                                           (holds owner "nav" only while driving)
    Align(robot, target)                   visual_align to a detector prompt
    PolicyStep(robot, label, instruction,  run an arm policy via policy_runner until
               policy_path='')             the arm is finished
    Holding(robot, name, holding=True)     condition on grasp_monitor: holding (a
                                           pick worked) / not holding (a place let go)

Each step keeps the contract of the building block it drives (docs/
omniman_vla.md, part 2): it waits until the service exists before sending,
follows the latched status the way the contract says, cancels what it started
when the tree interrupts it, and writes its reason next to itself in the
printed tree ("aligned in 7.4s, ..." / "empty - closed on nothing ...").

run_mission() does the rest: ROS init with a Ctrl+C that still cleans up,
mission.yaml and poses.yaml, a check that every Align target is a prompt of
the running detector, waiting for Nav2, the tick loop, printing the tree, and
stopping Nav2, visual_align and the policy and releasing "nav" on any exit.

Settings come from mission.yaml `settings:` (tick_s, service_wait_s,
still_time_s, still_linear, still_angular, settle_timeout_s); places from
poses.yaml next to it. Needs py_trees: sudo apt install ros-jazzy-py-trees
"""

import math
import os
import time

import py_trees
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from py_trees.common import Status
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

# Not settings - part of the contract with other nodes:
#  - the owner name for driving: control_arbiter's other users (the web UI's
#    lock display, other missions) know this robot's driver as "nav".
NAV = 'nav'
#  - the QoS the latched topics (/control/owner, the statuses, the gripper
#    state) are published with; a subscriber must use the same to get them.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Internal: messages handled per tick at most. Odometry alone arrives faster
# than the tick rate; reading one message per tick lets them pile up and the
# steps judge stale data. Nothing to tune.
SPIN_PER_TICK = 50

# mission.yaml `settings:` keys this module needs.
SETTINGS = ['tick_s', 'service_wait_s', 'still_time_s', 'still_linear', 'still_angular',
            'settle_timeout_s']


# ---- places ---------------------------------------------------------------

def load_poses(mission_file):
    """Places from poses.yaml beside the mission file (saved from the web UI)."""
    path = os.path.join(os.path.dirname(os.path.realpath(mission_file)), 'poses.yaml')
    with open(path) as f:
        return yaml.safe_load(f) or {}


def make_pose(node, pose_cfg, frame_id='map'):
    """Planar pose from a {x, y, yaw} dict; yaw in degrees."""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(pose_cfg['x'])
    p.pose.position.y = float(pose_cfg['y'])
    yaw = math.radians(float(pose_cfg['yaw']))
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


# ---- what the steps share ---------------------------------------------------

class Robot:
    """Everything the steps share: the ROS node, service clients and the
    latest value of each topic they watch. Created by run_mission() and handed
    to build(robot)."""

    def __init__(self, nav, cfg):
        self.nav = nav
        self.cfg = cfg
        self.settings = cfg['settings']
        self.owner = ''
        self.policy_status = ''
        self.align_status = ''
        self.holding = False
        self.grasp_state = 'no reading from grasp_monitor'
        self.still_since = None
        self.last_odom = 0.0
        self.pose = (0.0, 0.0, 0.0)     # x, y, yaw from odometry
        self.twist = (0.0, 0.0, 0.0)    # latest vx, vy, wz from odometry

        nav.create_subscription(ControlOwner, '/control/owner',
                                lambda m: setattr(self, 'owner', m.owner), LATCHED)
        nav.create_subscription(String, '/policy_runner/status',
                                lambda m: setattr(self, 'policy_status', m.data), LATCHED)
        nav.create_subscription(String, '/visual_align/status',
                                lambda m: setattr(self, 'align_status', m.data), LATCHED)
        nav.create_subscription(Bool, '/gripper/holding',
                                lambda m: setattr(self, 'holding', m.data), LATCHED)
        nav.create_subscription(String, '/grasp_monitor/state',
                                lambda m: setattr(self, 'grasp_state', m.data), LATCHED)
        nav.create_subscription(Odometry, '/mecanum_drive_controller/odometry',
                                self._on_odom, 10)
        self.target_pub = nav.create_publisher(String, '/visual_align/target', LATCHED)

        self.acquire = nav.create_client(AcquireControl, '/control/acquire')
        self.release_client = nav.create_client(ReleaseControl, '/control/release')
        self.align_run = nav.create_client(Trigger, '/visual_align/run')
        self.align_stop = nav.create_client(Trigger, '/visual_align/stop')
        self.policy_run = nav.create_client(RunPolicy, '/policy_runner/run')
        self.policy_stop = nav.create_client(Trigger, '/policy_runner/stop')

    def _on_odom(self, msg):
        t = msg.twist.twist
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        now = time.monotonic()
        self.last_odom = now
        self.twist = (t.linear.x, t.linear.y, t.angular.z)
        lin, ang = float(self.settings['still_linear']), float(self.settings['still_angular'])
        moving = abs(t.linear.x) >= lin or abs(t.linear.y) >= lin or abs(t.angular.z) >= ang
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def base_still(self):
        """True once odometry has shown the base stopped for still_time_s."""
        now = time.monotonic()
        return (now - self.last_odom < 0.5 and self.still_since is not None
                and now - self.still_since >= float(self.settings['still_time_s']))

    def release(self):
        req = ReleaseControl.Request()
        req.owner = NAV
        self.release_client.call_async(req)


class Step(py_trees.behaviour.Behaviour):
    """Base class for a step that drives a building block: start it
    (initialise / the first ticks), follow it (update, every tick), cancel it
    when interrupted (terminate). Subclass it for a new building block."""

    def __init__(self, name, robot):
        super().__init__(name)
        self.robot = robot
        self.log = robot.nav.get_logger()
        self.since = time.monotonic()

    def fail(self, reason):
        """FAILURE, with the reason shown next to the step in the tree."""
        self.feedback_message = reason
        self.log.error(f'{self.name}: {reason}')
        return Status.FAILURE

    def succeed(self, note):
        """SUCCESS, with a note shown next to the step in the tree."""
        self.feedback_message = note
        self.log.info(f'{self.name}: {note}')
        return Status.SUCCESS

    def interrupted(self, new_status):
        """For terminate(): True only if the step was still RUNNING when the
        tree stopped it - terminate() also runs after SUCCESS / FAILURE."""
        return new_status == Status.INVALID and self.status == Status.RUNNING

    def send(self, client, request):
        """Send a request once the service is there - one sent before it is
        discovered can be lost. None while waiting, 'gone' after
        service_wait_s (reason set), else the future."""
        if client.service_is_ready():
            return client.call_async(request)
        if time.monotonic() - self.since > float(self.robot.settings['service_wait_s']):
            self.fail(f'{client.srv_name} not answering - is control_launch.py running?')
            return 'gone'
        return None

    def settle(self, since):
        """RUNNING until the base is still, FAILURE past settle_timeout_s."""
        r = self.robot
        if r.base_still():
            return Status.SUCCESS
        if time.monotonic() - since > float(r.settings['settle_timeout_s']):
            if time.monotonic() - r.last_odom > 0.5:
                return self.fail('base did not settle: no odometry for '
                                 f'{time.monotonic() - r.last_odom:.1f}s')
            vx, vy, wz = r.twist
            return self.fail(f'base did not settle: odometry still moving (vx {vx:+.3f}, '
                             f'vy {vy:+.3f} m/s, wz {wz:+.3f} rad/s; still = under '
                             f'{r.settings["still_linear"]} m/s and '
                             f'{r.settings["still_angular"]} rad/s)')
        return Status.RUNNING


# ---- the steps --------------------------------------------------------------

class Navigate(Step):
    """Drive to a place in poses.yaml: acquire "nav", drive with Nav2, wait
    until the base is still, release. Cancelled (and released) if interrupted
    or if control is taken away."""

    def __init__(self, robot, place):
        super().__init__(f'nav to {place}', robot)
        self.place = place

    def initialise(self):
        self.phase, self.future, self.since, self.noted = 'acquire', None, time.monotonic(), 0.0
        self.feedback_message = ''

    def update(self):
        r = self.robot
        if self.phase == 'acquire':
            if self.future is None:
                req = AcquireControl.Request()
                req.owner = NAV
                req.node = r.nav.get_fully_qualified_name()
                self.future = self.send(r.acquire, req)
                if self.future == 'gone':
                    return Status.FAILURE
            elif self.future.done():
                res, self.future = self.future.result(), None
                if res is not None and res.success:
                    self.phase, self.since = 'owner', time.monotonic()
                else:
                    self.feedback_message = ('waiting for control: '
                                             f'{res.message if res else "no answer"}')
                    if time.monotonic() - self.noted > 5.0:
                        self.log.info(f'{self.name}: {self.feedback_message}')
                        self.noted = time.monotonic()
            return Status.RUNNING
        if self.phase == 'owner':
            # /control/owner must say so too, or a takeover check reads stale.
            if r.owner == NAV or time.monotonic() - self.since > 3.0:
                pose = r.cfg['poses'][self.place]
                self.log.info(f'{self.name} (x={pose["x"]:.2f}, y={pose["y"]:.2f}, '
                              f'yaw={pose["yaw"]:.0f})')
                r.nav.goToPose(make_pose(r.nav, pose))
                self.phase = 'drive'
            return Status.RUNNING
        if self.phase == 'drive':
            if r.owner != NAV:
                r.nav.cancelTask()
                self.phase = 'done'
                return self.fail(f'control taken by {r.owner or "a force release"}')
            if not r.nav.isTaskComplete():
                return Status.RUNNING
            result = r.nav.getResult()
            if result != TaskResult.SUCCEEDED:
                r.release()
                self.phase = 'done'
                return self.fail(f'Nav2 did not reach it ({getattr(result, "name", result)})')
            self.phase, self.since = 'settle', time.monotonic()
            return Status.RUNNING
        status = self.settle(self.since)
        if status != Status.RUNNING:
            r.release()
            self.phase = 'done'
        if status == Status.SUCCESS:
            return self.succeed('arrived')
        return status

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase in ('owner', 'drive', 'settle'):
            self.robot.nav.cancelTask()
            self.robot.release()
            self.log.warn(f'{self.name}: interrupted - navigation cancelled')


class Align(Step):
    """visual_align to `target` - exactly a prompt of the running detector -
    then wait for the base to be still. Stopped if interrupted."""

    def __init__(self, robot, target):
        super().__init__(f'align to "{target}"', robot)
        self.target = target

    def initialise(self):
        self.robot.target_pub.publish(String(data=self.target))
        self.future = None
        self.phase, self.busy_seen, self.since = 'call', False, time.monotonic()
        self.start_pose = self.robot.pose
        self.feedback_message = ''

    def update(self):
        r = self.robot
        if self.phase == 'call':
            if self.future is None:
                self.future = self.send(r.align_run, Trigger.Request())
                if self.future == 'gone':
                    return Status.FAILURE
                return Status.RUNNING
            if not self.future.done():
                return Status.RUNNING
            res = self.future.result()
            if res is None or not res.success:
                return self.fail(f'visual_align refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'watch', time.monotonic()
            return Status.RUNNING
        if self.phase == 'watch':
            # aligned / failed stay latched: only a result after having been
            # busy is this run's (or after 3 s, if the busy states were missed).
            s = r.align_status
            done = s == 'aligned' or s.startswith('failed')
            if not done:
                self.busy_seen = True
                self.feedback_message = s
                return Status.RUNNING
            if not self.busy_seen and time.monotonic() - self.since < 3.0:
                return Status.RUNNING
            if s != 'aligned':
                self.phase = 'done'
                return self.fail(s)
            self.phase, self.took = 'settle', time.monotonic() - self.since
            self.since = time.monotonic()
            return Status.RUNNING
        status = self.settle(self.since)
        if status == Status.SUCCESS:
            (x0, y0, a0), (x1, y1, a1) = self.start_pose, r.pose
            turned = math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
            moved = math.hypot(x1 - x0, y1 - y0)
            self.phase = 'done'
            return self.succeed(f'aligned in {self.took:.1f}s, base turned {turned:+.0f} deg, '
                                f'moved {moved:.2f} m')
        return status

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase == 'watch':
            self.robot.align_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - align stopped')


class PolicyStep(Step):
    """Run an arm policy through policy_runner until the arm is finished (back
    home, see policy_runner.py). policy_path empty = policy_runner's default.
    Stopped if interrupted."""

    def __init__(self, robot, label, instruction, policy_path=''):
        super().__init__(f'{label} policy', robot)
        self.instruction = instruction
        self.policy_path = os.path.expanduser(policy_path) if policy_path else ''

    def initialise(self):
        self.future = None
        self.phase, self.busy_seen, self.since = 'call', False, time.monotonic()
        self.feedback_message = ''
        self.log.info(f'{self.name}: "{self.instruction}"')

    def update(self):
        r = self.robot
        if self.phase == 'call':
            if self.future is None:
                req = RunPolicy.Request()
                req.policy_path = self.policy_path
                req.instruction = self.instruction
                req.force = False
                self.future = self.send(r.policy_run, req)
                if self.future == 'gone':
                    return Status.FAILURE
                return Status.RUNNING
            if not self.future.done():
                return Status.RUNNING
            res = self.future.result()
            if res is None or not res.success:
                return self.fail(f'policy_runner refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'watch', time.monotonic()
            return Status.RUNNING
        # starting -> working -> idle; idle after having been busy ends it.
        if r.policy_status != 'idle':
            self.busy_seen = True
            self.feedback_message = f'{r.policy_status} {time.monotonic() - self.since:.0f}s'
            return Status.RUNNING
        if self.busy_seen or time.monotonic() - self.since > 5.0:
            self.phase = 'done'
            return self.succeed(f'arm back home after {time.monotonic() - self.since:.0f}s')
        return Status.RUNNING

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase == 'watch':
            self.robot.policy_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - policy stopped')


class Holding(py_trees.behaviour.Behaviour):
    """Condition on grasp_monitor: SUCCESS when the gripper holds something
    (holding=True: a pick worked) or, with holding=False, when it no longer
    does (a place let go)."""

    def __init__(self, robot, name, holding=True):
        super().__init__(name)
        self.robot = robot
        self.want = holding

    def update(self):
        # grasp_state carries the measured values, e.g.
        # "empty - closed on nothing (position -0.0120, effort -1)".
        self.feedback_message = self.robot.grasp_state
        if self.robot.holding == self.want:
            self.robot.nav.get_logger().info(f'{self.name}: {self.feedback_message}')
            return Status.SUCCESS
        self.robot.nav.get_logger().warn(f'{self.name}: no - {self.feedback_message}')
        return Status.FAILURE


# ---- running a mission ---------------------------------------------------------

def check_align_targets(root):
    """Every Align in the tree must name exactly a prompt of the detector
    visual_align listens to: detections carry the prompt as their name, and
    visual_align accepts only the current target, so a target spelled
    differently ("black square" vs "black rectangle") is never found and the
    run fails after a full search turn. Returns what is wrong, or ''."""
    targets = sorted({b.target for b in root.iterate() if isinstance(b, Align)})
    if not targets:
        return ''
    path = f"{get_package_share_directory('omniman_vla')}/config/visual_align.yaml"
    with open(path) as f:
        va = yaml.safe_load(f) or {}
    topic = va.get('visual_align', {}).get('detections_topic', '')
    detector = topic.strip('/').split('/')[0]        # /efficient_sam_detector/detections
    prompts = list(va.get(detector, {}).get('prompts') or [])
    if not prompts:
        return (f'cannot check the align targets: no prompts for "{detector}" (from '
                f'detections_topic {topic}) in {path}')
    bad = [t for t in targets if t not in prompts]
    if bad:
        return (f'align target(s) {bad} not among {detector}\'s prompts {prompts} - '
                'fix the mission or the prompts in visual_align.yaml')
    return ''


def call_sync(nav, client, request, timeout_s=5.0):
    """A blocking service call - for shutdown only, never inside a step."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(nav, future, timeout_sec=timeout_s)
    return future.result()


def run_mission(build, node_name, initial_pose='home'):
    """Run the tree build(robot) returns until it succeeds or fails.

    build         function robot -> root behaviour of the mission's tree
    node_name     ROS node name of the mission
    initial_pose  place in poses.yaml to give AMCL at start (the robot is
                  assumed to stand there when launched); None = keep AMCL's

    The mission file is the `mission_file` parameter (default:
    config/mission.yaml); poses.yaml is read from beside it. Returns True
    if the mission succeeded.
    """
    # rclpy's SIGINT handler would shut the context down before the steps
    # could stop what they started; Python's default raises KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator(node_name=node_name)
    log = nav.get_logger()

    default_cfg = f"{get_package_share_directory('omniman_vla')}/config/mission.yaml"
    nav.declare_parameter('mission_file', default_cfg)
    mission_file = nav.get_parameter('mission_file').value
    with open(mission_file) as f:
        cfg = yaml.safe_load(f)
    cfg['poses'] = load_poses(mission_file)
    log.info(f'mission: {mission_file}')
    missing = [k for k in SETTINGS if k not in (cfg.get('settings') or {})]
    if missing:
        log.error(f'{mission_file} settings: missing {missing}')
        nav.destroy_node()
        rclpy.try_shutdown()
        return False

    robot = Robot(nav, cfg)
    root = build(robot)
    problem = check_align_targets(root)
    if problem:
        log.error(problem)
        nav.destroy_node()
        rclpy.try_shutdown()
        return False

    if initial_pose is not None:
        # BasicNavigator's default initial pose is a zero-norm quaternion, and
        # waitUntilNav2Active() publishes it until AMCL answers - so a real one
        # must be set, or a good AMCL estimate is clobbered.
        nav.setInitialPose(make_pose(nav, cfg['poses'][initial_pose]))
    log.info('waiting for Nav2...')
    nav.waitUntilNav2Active()

    tree = py_trees.trees.BehaviourTree(root)
    log.info('\n' + py_trees.display.unicode_tree(root))
    tick_s = float(cfg['settings']['tick_s'])
    shown = ''
    try:
        while True:
            start = time.monotonic()
            tree.tick()
            view = py_trees.display.unicode_tree(root, show_status=True)
            if view != shown:
                log.info('\n' + view)
                shown = view
            if root.status in (Status.SUCCESS, Status.FAILURE):
                break
            # Handle every message waiting, not just one; each call returns
            # at once when nothing is waiting.
            for _ in range(SPIN_PER_TICK):
                rclpy.spin_once(nav, timeout_sec=0.0)
            time.sleep(max(0.0, tick_s - (time.monotonic() - start)))
        if root.status == Status.SUCCESS:
            log.info('mission complete')
        else:
            log.error('mission ABORTED')
    except KeyboardInterrupt:
        log.warn('interrupted - stopping everything')
        root.stop(Status.INVALID)
    finally:
        # Never leave the robot driving, aligning, running a policy or
        # holding "nav"; each stop is a no-op when that part is idle.
        nav.cancelTask()
        call_sync(nav, robot.align_stop, Trigger.Request())
        call_sync(nav, robot.policy_stop, Trigger.Request())
        if robot.owner == NAV:
            req = ReleaseControl.Request()
            req.owner = NAV
            call_sync(nav, robot.release_client, req)
        nav.destroy_node()
        rclpy.try_shutdown()
    return root.status == Status.SUCCESS
