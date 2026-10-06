#!/usr/bin/env python3
"""
omniman's steps as nodes for Cyclo Intelligence's behaviour-tree engine
(Autonomy Studio, Action Canvas). Each class below is one node in the Studio
palette; its constructor arguments are the fields the UI shows.

They drive omniman's own nodes - the same ones the py_trees missions use
(omniman_vla.mission) - so both kinds of mission behave the same:

    OmnimanNavigate     Nav2 to a place in omniman_vla/config/poses.yaml,
                        holding the control lock as "nav" while driving
    OmnimanAlign        base correction: visual_align on what the SAM detector
                        finds for `target` (any text)
    OmnimanPolicy       an arm policy through policy_runner (physical_ai_server:
                        the ACT policies trained with physical_ai_tools), until
                        the arm is home
    OmnimanArmHome      waits until the arm is home for dwell_s: the end of a
                        run started with Cyclo's own SendCommand (GR00T,
                        LeRobot 0.6), which has no "finished" of its own
    OmnimanGraspCheck   grasp_monitor: is the gripper holding (or not)
    StartAgain          always fails - last in an OnFailure recovery, to make an
                        enclosing Attempts run everything again

The controls that give a tree retries and recovery (Attempts, OnFailure,
WhileHolding, WithControlLock) are in omniman_controls.py.

Linked into src/cyclo_intelligence/orchestrator/orchestrator/bt/actions/,
where the engine finds its own nodes.
Settings come from omniman's files, read when a tree is loaded:
omniman_vla/config/poses.yaml (places), mission.yaml (when the base counts as
still), policy_runner.yaml (the arm's home pose).

The engine stops a tree by resetting every node, so a node that started
something - a Nav2 goal, visual_align, a policy - cancels it in reset().
"""

import math
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.bt_core import NodeStatus
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

# omniman's src/ (the git repo) - set by the launch file; else found from here:
# <src>/cyclo_intelligence/orchestrator/orchestrator/bt/actions/this file.
SRC = Path(os.environ.get('OMNIMAN_SRC') or Path(__file__).resolve().parents[5])
CONFIG = SRC / 'omniman_vla' / 'config'

ARM_JOINTS = ['shoulder_yaw_joint', 'upper_shoulder_pitch_joint', 'arm_yaw_joint',
              'forearm_pitch_joint', 'wrist_pitch_joint', 'palm_yaw_joint']
ODOM_TOPIC = '/mecanum_drive_controller/odometry'
SERVICE_WAIT_S = 10.0      # a service that does not appear by then fails the node

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

RUNNING, SUCCESS, FAILURE = NodeStatus.RUNNING, NodeStatus.SUCCESS, NodeStatus.FAILURE


def _text(value):
    """An XML text field as a string: the engine turns "a, b" into a list."""
    if isinstance(value, (list, tuple)):
        return ', '.join(str(v) for v in value)
    return str(value)


def _read_yaml(name):
    with open(CONFIG / name) as f:
        return yaml.safe_load(f) or {}


class _Omniman:
    """What every omniman node in one engine needs, made once: the topics
    they read, the services they call. The engine is one ROS node; this
    subscribes on it."""

    def __init__(self, node):
        self.node = node
        settings = _read_yaml('mission.yaml').get('settings') or {}
        self.still_time_s = float(settings.get('still_time_s', 0.5))
        self.still_linear = float(settings.get('still_linear', 0.01))
        self.still_angular = float(settings.get('still_angular', 0.02))

        self.align_status = ''
        self.holding = False
        self.gripper = 'no reading from grasp_monitor'
        self.policy_status = ''
        self.arm = 'no reading from policy_runner'
        self.owner = ''
        self.twist = (0.0, 0.0, 0.0)
        self.have_odom = False
        self.still_since = None
        self.joints = {}

        def keep(attr, convert=lambda m: m.data):
            return lambda msg: setattr(self, attr, convert(msg))

        node.create_subscription(String, '/visual_align/status', keep('align_status'), LATCHED)
        node.create_subscription(Bool, '/gripper/holding', keep('holding'), LATCHED)
        node.create_subscription(String, '/grasp_monitor/state', keep('gripper'), LATCHED)
        node.create_subscription(String, '/policy_runner/status', keep('policy_status'), LATCHED)
        node.create_subscription(String, '/policy_runner/arm', keep('arm'), LATCHED)
        node.create_subscription(ControlOwner, '/control/owner',
                                 keep('owner', lambda m: m.owner), LATCHED)
        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        node.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        self.target_pub = node.create_publisher(String, '/visual_align/target', LATCHED)
        self.align_run = node.create_client(Trigger, '/visual_align/run')
        self.align_stop = node.create_client(Trigger, '/visual_align/stop')
        self.policy_run = node.create_client(RunPolicy, '/policy_runner/run')
        self.policy_stop = node.create_client(Trigger, '/policy_runner/stop')
        self.acquire = node.create_client(AcquireControl, '/control/acquire')
        self.release_client = node.create_client(ReleaseControl, '/control/release')
        self.nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def _on_odom(self, msg):
        t = msg.twist.twist
        self.twist = (t.linear.x, t.linear.y, t.angular.z)
        self.have_odom = True
        still = (abs(t.linear.x) < self.still_linear and abs(t.linear.y) < self.still_linear
                 and abs(t.angular.z) < self.still_angular)
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = time.monotonic()

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def base_still(self):
        return (self.still_since is not None
                and time.monotonic() - self.still_since >= self.still_time_s)

    def release(self, owner):
        if self.release_client.service_is_ready():
            req = ReleaseControl.Request()
            req.owner = owner
            self.release_client.call_async(req)


_SHARED = {}


def omniman(node):
    """The _Omniman for this engine node (made on first use)."""
    if id(node) not in _SHARED:
        _SHARED[id(node)] = _Omniman(node)
    return _SHARED[id(node)]


class _Step:
    """Helpers the omniman nodes share. Not a BaseAction itself, so the
    engine does not list it as a node."""

    def _start(self, node):
        self.om = omniman(node)
        self._waiting_since = None

    def call(self, client, request):
        """A future once the service exists; None while waiting for it;
        'gone' if it has not appeared after SERVICE_WAIT_S. A request sent
        before the service is discovered can be lost, so never send blind."""
        if self._waiting_since is None:
            self._waiting_since = time.monotonic()
        if client.service_is_ready():
            self._waiting_since = None
            return client.call_async(request)
        if time.monotonic() - self._waiting_since > SERVICE_WAIT_S:
            self._waiting_since = None
            return 'gone'
        return None

    def succeed(self, note):
        self.log_info(note)
        return SUCCESS

    def fail(self, reason):
        self.log_warn(reason)
        return FAILURE

    def settle(self, since, timeout_s):
        """RUNNING until odometry shows the base still, FAILURE after timeout_s."""
        if self.om.base_still():
            return SUCCESS
        if time.monotonic() - since > timeout_s:
            if not self.om.have_odom:
                return self.fail(f'base did not settle: no odometry on {ODOM_TOPIC}')
            vx, vy, wz = self.om.twist
            return self.fail(f'base did not settle: vx {vx:+.3f} vy {vy:+.3f} m/s, '
                             f'wz {wz:+.3f} rad/s')
        return RUNNING


class OmnimanNavigate(_Step, BaseAction):
    """Drive with Nav2 to a place saved in omniman_vla/config/poses.yaml
    (the web UI's "Save here"). Takes the control lock as "nav" while
    driving, waits for the base to be still, gives the lock back."""

    def __init__(self, node, place: str = 'pick_area', settle_timeout_s: float = 5.0,
                 timeout_s: float = 0.0):
        super().__init__(node, name='OmnimanNavigate')
        self._start(node)
        self.place = _text(place)
        self.settle_timeout_s = float(settle_timeout_s)
        self.timeout_s = float(timeout_s)         # 0 = until Nav2 says
        self._clear()

    def _clear(self):
        self.phase, self.future, self.goal, self.result = 'acquire', None, None, None
        self.since = time.monotonic()
        self.noted = 0.0

    def tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'acquire':
            if self.future is None:
                req = AcquireControl.Request()
                req.owner = 'nav'
                req.node = self.node.get_fully_qualified_name()
                self.future = self.call(om.acquire, req)
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('control_arbiter not answering (/control/acquire)')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res, self.future = self.future.result(), None
            if res is None or not res.success:
                if now - self.noted > 5.0:      # someone else drives: wait, ask again
                    self.log_info(f'waiting for control: {res.message if res else "no answer"}')
                    self.noted = now
                return RUNNING
            self.phase, self.since = 'owner', now
            return RUNNING

        if self.phase == 'owner':
            # /control/owner must say so too, or the takeover check below
            # would read an older "nobody".
            if om.owner == 'nav' or now - self.since > 3.0:
                self.phase, self.since = 'goal', now
            return RUNNING

        if self.phase == 'goal':
            if not om.nav.server_is_ready():
                if now - self.since > SERVICE_WAIT_S:
                    self._end()
                    return self.fail('Nav2 not answering (navigate_to_pose)')
                return RUNNING
            try:
                pose = _read_yaml('poses.yaml')[self.place]
            except (OSError, KeyError) as e:
                self._end()
                return self.fail(f'no place "{self.place}" in {CONFIG / "poses.yaml"} ({e})')
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(pose['x'])
            goal.pose.pose.position.y = float(pose['y'])
            yaw = math.radians(float(pose['yaw']))
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.log_info(f'nav to {self.place} (x={pose["x"]:.2f}, y={pose["y"]:.2f}, '
                          f'yaw={pose["yaw"]:.0f})')
            self.future = om.nav.send_goal_async(goal)
            self.phase, self.since = 'accepted', now
            return RUNNING

        if self.phase == 'accepted':
            if not self.future.done():
                return RUNNING
            self.goal = self.future.result()
            if self.goal is None or not self.goal.accepted:
                self._end()
                return self.fail('Nav2 refused the goal')
            self.result = self.goal.get_result_async()
            self.phase = 'drive'
            return RUNNING

        if self.phase == 'drive':
            if om.owner != 'nav':
                self._end()
                return self.fail(f'control taken by {om.owner or "a forced release"}')
            if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
                self._end()
                return self.fail(f'Nav2 did not arrive within {self.timeout_s:.0f}s')
            if not self.result.done():
                return RUNNING
            status = self.result.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._end()
                return self.fail(f'Nav2 did not reach {self.place} (goal status {status})')
            self.phase, self.since = 'settle', now
            return RUNNING

        if self.phase == 'settle':
            status = self.settle(self.since, self.settle_timeout_s)
            if status == RUNNING:
                return RUNNING
            self._end()
            return self.succeed(f'arrived at {self.place}') if status == SUCCESS else status
        return FAILURE

    def _end(self):
        """Cancel a goal still running, give the lock back - also for an
        answer still on its way, so a late "acquired" or "goal accepted"
        cannot leave the lock held or the base driving."""
        om, future = self.om, self.future
        if self.phase == 'acquire' and future not in (None, 'gone') and not future.done():
            future.add_done_callback(
                lambda f: f.result() is not None and f.result().success and om.release('nav'))
        if self.phase == 'accepted' and not future.done():
            def cancel_late(f):
                goal = f.result()
                if goal is not None and goal.accepted:
                    goal.cancel_goal_async()
                om.release('nav')
            future.add_done_callback(cancel_late)
        elif self.phase == 'drive' and self.result is not None and not self.result.done():
            self.goal.cancel_goal_async()
        if self.phase in ('owner', 'goal', 'drive', 'settle'):
            om.release('nav')
        self.phase = 'done'

    def reset(self):
        super().reset()
        if self.phase not in ('acquire', 'done'):
            self.log_warn('stopped - navigation cancelled')
        self._end()
        self._clear()


class OmnimanAlign(_Step, BaseAction):
    """Base correction: visual_align moves the base until the detector's
    `target` sits where the policy expects it (aim point in
    visual_align.yaml), then waits for the base to be still. `target` is any
    text - it is the SAM detector's prompt, e.g. "yellow cup lid"."""

    def __init__(self, node, target: str = 'yellow cup lid', timeout_s: float = 60.0,
                 settle_timeout_s: float = 5.0):
        super().__init__(node, name='OmnimanAlign')
        self._start(node)
        self.target = _text(target)
        self.timeout_s = float(timeout_s)         # 0 = until visual_align says
        self.settle_timeout_s = float(settle_timeout_s)
        self._clear()

    def _clear(self):
        self.phase, self.future, self.busy_seen = 'start', None, False
        self.since = time.monotonic()

    def tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'start':
            om.target_pub.publish(String(data=self.target))
            self.log_info(f'align to "{self.target}"')
            self.phase, self.since = 'call', now
        if self.phase == 'call':
            if self.future is None:
                self.future = self.call(om.align_run, Trigger.Request())
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('visual_align not answering (/visual_align/run)')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res = self.future.result()
            if res is None or not res.success:
                self.phase = 'done'
                return self.fail(f'visual_align refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'align', now
            return RUNNING

        if self.phase == 'align':
            # The status is latched: an old "aligned" is there before this
            # run starts, so only a result after busy counts.
            status = om.align_status
            if status in ('starting', 'searching', 'aligning'):
                self.busy_seen = True
            elif self.busy_seen and status == 'aligned':
                self.log_info(f'aligned in {now - self.since:.1f}s')
                self.phase, self.since = 'settle', now
                return RUNNING
            elif self.busy_seen and status.startswith('failed'):
                self.phase = 'done'
                return self.fail(f'visual_align {status}')
            if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
                self._stop()
                return self.fail(f'no result within {self.timeout_s:.0f}s '
                                 f'(last: {status or "nothing"}) - align stopped')
            return RUNNING

        if self.phase == 'settle':
            status = self.settle(self.since, self.settle_timeout_s)
            if status != RUNNING:
                self.phase = 'done'
            return status
        return FAILURE

    def _stop(self):
        if self.phase == 'align' and self.om.align_stop.service_is_ready():
            self.om.align_stop.call_async(Trigger.Request())
        self.phase = 'done'

    def reset(self):
        super().reset()
        if self.phase == 'align':
            self.log_warn('stopped - align stopped')
        self._stop()
        self._clear()


class OmnimanPolicy(_Step, BaseAction):
    """Run an arm policy through omniman's policy_runner (physical_ai_server -
    the ACT policies trained with physical_ai_tools) until the arm is back
    home. `policy_path` empty = policy_runner.yaml's default. timeout_s
    stops a policy that never settles at home."""

    def __init__(self, node, instruction: str = 'pick the object', policy_path: str = '',
                 timeout_s: float = 90.0):
        super().__init__(node, name='OmnimanPolicy')
        self._start(node)
        self.instruction = _text(instruction)
        self.policy_path = os.path.expanduser(_text(policy_path)) if policy_path else ''
        self.timeout_s = float(timeout_s)         # 0 = until policy_runner says
        self._clear()

    def _clear(self):
        self.phase, self.future, self.busy_seen = 'call', None, False
        self.since = time.monotonic()

    def tick(self):
        om, now = self.om, time.monotonic()
        if self.phase == 'call':
            if self.future is None:
                req = RunPolicy.Request()
                req.policy_path = self.policy_path
                req.instruction = self.instruction
                req.force = False
                self.future = self.call(om.policy_run, req)
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('policy_runner not answering (/policy_runner/run)')
                if self.future is not None:
                    self.log_info(f'policy: "{self.instruction}"')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res = self.future.result()
            if res is None or not res.success:
                self.phase = 'done'
                return self.fail(f'policy_runner refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'watch', now
            return RUNNING

        if self.phase == 'watch':
            if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
                self._stop()
                return self.fail(f'policy ran longer than {self.timeout_s:.0f}s '
                                 f'({om.arm}) - policy stopped')
            # starting -> working -> idle; idle after busy is the end. The
            # status is latched, so an idle before that is the last run's.
            if om.policy_status != 'idle':
                self.busy_seen = True
                return RUNNING
            if self.busy_seen or now - self.since > 5.0:
                self.phase = 'done'
                return self.succeed(f'arm back home after {now - self.since:.0f}s')
            return RUNNING
        return FAILURE

    def _stop(self):
        if self.phase == 'watch' and self.om.policy_stop.service_is_ready():
            self.om.policy_stop.call_async(Trigger.Request())
        self.phase = 'done'

    def reset(self):
        super().reset()
        if self.phase == 'watch':
            self.log_warn('stopped - policy stopped')
        self._stop()
        self._clear()


class OmnimanArmHome(_Step, BaseAction):
    """Wait until the arm is at home - every arm joint within `tolerance`
    rad of home_pose in policy_runner.yaml - for dwell_s in a row. Use it
    after Cyclo's SendCommand RESUME to know a GR00T / LeRobot 0.6 run is
    done (then SendCommand STOP). Fails after timeout_s."""

    def __init__(self, node, dwell_s: float = 7.0, tolerance: float = 0.2,
                 timeout_s: float = 90.0):
        super().__init__(node, name='OmnimanArmHome')
        self._start(node)
        self.dwell_s = float(dwell_s)
        self.tolerance = float(tolerance)
        self.timeout_s = float(timeout_s)         # 0 = wait for ever
        params = (_read_yaml('policy_runner.yaml').get('policy_runner') or {}) \
            .get('ros__parameters') or {}
        self.home = [float(v) for v in params.get('home_pose', [])]
        if len(self.home) != len(ARM_JOINTS):
            raise ValueError(f'OmnimanArmHome: home_pose in {CONFIG / "policy_runner.yaml"} '
                             f'must have {len(ARM_JOINTS)} values')
        self._clear()

    def _clear(self):
        self.since = time.monotonic()
        self.home_since = None
        self.noted = 0.0

    def tick(self):
        now, joints = time.monotonic(), self.om.joints
        if any(j not in joints for j in ARM_JOINTS):
            if now - self.since > SERVICE_WAIT_S:
                return self.fail('no arm joints on /joint_states')
            return RUNNING
        worst = max(abs(joints[j] - h) for j, h in zip(ARM_JOINTS, self.home))
        if worst > self.tolerance:
            self.home_since = None
        elif self.home_since is None:
            self.home_since = now
        if self.home_since is not None and now - self.home_since >= self.dwell_s:
            return self.succeed(f'arm home for {self.dwell_s:g}s (worst joint {worst:.3f} rad)')
        if self.timeout_s > 0.0 and now - self.since > self.timeout_s:
            return self.fail(f'arm not home for {self.dwell_s:g}s within '
                             f'{self.timeout_s:.0f}s (worst joint {worst:.3f} rad)')
        return RUNNING

    def reset(self):
        super().reset()
        self._clear()


class OmnimanGraspCheck(_Step, BaseAction):
    """grasp_monitor: SUCCESS when the gripper holds something
    (holding=true: a pick worked) or, with holding=false, when it does not
    (a place let go). within_s gives a gripper still moving that long to get
    there; 0 = read it once."""

    def __init__(self, node, holding: bool = True, within_s: float = 0.0):
        super().__init__(node, name='OmnimanGraspCheck')
        self._start(node)
        self.want = bool(holding)
        self.within_s = float(within_s)
        self.since = time.monotonic()
        self.first = True

    def tick(self):
        if self.first:
            self.since, self.first = time.monotonic(), False
        if self.om.holding == self.want:
            return self.succeed(self.om.gripper)
        if time.monotonic() - self.since < self.within_s:
            return RUNNING
        return self.fail(f'{"not holding" if self.want else "still holding"}: '
                         f'{self.om.gripper}')

    def reset(self):
        super().reset()
        self.first = True


class StartAgain(_Step, BaseAction):
    """Always fails. Last in an OnFailure recovery, it makes the Attempts
    around the whole mission start it again from the beginning."""

    def __init__(self, node, reason: str = 'start again'):
        super().__init__(node, name='StartAgain')
        self.reason = _text(reason)

    def tick(self):
        self.log_info(self.reason)
        return FAILURE
