#!/usr/bin/env python3
"""
What every omniman action shares: where omniman's settings are, the topics and
services one engine needs (_Omniman, made once per engine node) and the helper
methods of the nodes (_Step).

Settings come from omniman's files, read when a tree is loaded:
omniman_vla/config/poses.yaml (places), mission.yaml (when the base counts as
still), policy_runner.yaml (the arm's home pose).
"""

import math  # noqa: F401
import os
from pathlib import Path
import time

from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from orchestrator.bt.bt_core import NodeStatus
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

# omniman's src/ (the git repo) - set by the launch file; else found from here:
# <src>/cyclo_intelligence/orchestrator/orchestrator/bt/actions/omniman/this file.
SRC = Path(os.environ.get('OMNIMAN_SRC') or Path(__file__).resolve().parents[6])
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
