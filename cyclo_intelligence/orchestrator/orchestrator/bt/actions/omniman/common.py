#!/usr/bin/env python3
"""
What every omniman action shares: where omniman's settings are, the topics and
services one engine needs (_Omniman, made once per engine node) and the helper
methods of the nodes (_Step).

Settings come from omniman's files, read when a tree is loaded:
omniman_vla/config/poses.yaml (places), arm_poses.yaml (arm poses),
mission.yaml (when the base counts as still, the arm and look settings, the
grasp thresholds), policy_runner.yaml (the arm's home pose).

Nothing here waits a fixed time: a node moves on when a message says so (the
odometry is quiet, the arm controller says a move is finished, a camera frame
arrives after the robot settled, grasp_monitor agrees with the gripper), and
the time settings only decide when to give up. Times kept below are when
messages ARRIVED here (time.monotonic), not their own stamps: the camera's
header stamps are about 0.6 s early (measured: the picture reacts 0.04 s after
the joints).
"""

import math  # noqa: F401
import os
from pathlib import Path
import time

from control_msgs.msg import JointTrajectoryControllerState
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from orchestrator.bt.bt_core import NodeStatus
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
import yaml
from vision_msgs.msg import Detection2DArray

# omniman's src/ (the git repo) - set by the launch file; else found from here:
# <src>/cyclo_intelligence/orchestrator/orchestrator/bt/actions/omniman/this file.
SRC = Path(os.environ.get('OMNIMAN_SRC') or Path(__file__).resolve().parents[6])
CONFIG = SRC / 'omniman_vla' / 'config'

ARM_JOINTS = ['shoulder_yaw_joint', 'upper_shoulder_pitch_joint', 'arm_yaw_joint',
              'forearm_pitch_joint', 'wrist_pitch_joint', 'palm_yaw_joint']
GRIPPER_JOINT = 'gripper_prismatic_joint'
ARM = 'arm'                # the owner name for moving the arm to a pose
# Where arm moves go: the policy's path - the teleop relay passes it on to
# /arm_controller/joint_trajectory.
ARM_TOPIC = '/leader/joint_trajectory'
# The controller's trajectory counts as at its end when its reference is this
# close to the target (rad): it samples the last point exactly.
REFERENCE_AT_TARGET = 1e-3
ODOM_TOPIC = '/mecanum_drive_controller/odometry'
SERVICE_WAIT_S = 10.0      # a service that does not appear by then fails the node

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# The same, but keeping up to 10 messages between two ticks: a node waits for a
# status to go busy and then done, and a status that changes twice in one tick
# must not lose the busy one.
STATUS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
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
        mission = _read_yaml('mission.yaml')
        settings = mission.get('settings') or {}
        self.settings = settings
        self.grasp_cfg = mission.get('grasp_monitor') or {}
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
        self.last_odom = 0.0
        self.still_at = None         # when odometry first showed the base still, this stop
        self.joints = {}
        self.efforts = {}
        self.detections_seen = 0     # detector messages so far (empty ones too)
        self.detection_hits = 0      # ... of which had something in them
        self.last_hit = ('', 0.0)    # the latest hit: (prompt, score)
        self.camera_at = 0.0         # when the latest camera frame arrived
        self.arm_arrived_at = 0.0    # when the arm controller said a move had finished
        self.ctrl_reference = {}     # arm_controller: the trajectory point it is tracking
        self.ctrl_error = {}         # arm_controller: tracking error per joint

        def keep(attr, convert=lambda m: m.data):
            return lambda msg: setattr(self, attr, convert(msg))

        node.create_subscription(String, '/visual_align/status', keep('align_status'), STATUS)
        node.create_subscription(Bool, '/gripper/holding', keep('holding'), LATCHED)
        node.create_subscription(String, '/grasp_monitor/state', keep('gripper'), LATCHED)
        node.create_subscription(String, '/policy_runner/status', keep('policy_status'), STATUS)
        node.create_subscription(String, '/policy_runner/arm', keep('arm'), LATCHED)
        node.create_subscription(ControlOwner, '/control/owner',
                                 keep('owner', lambda m: m.owner), LATCHED)
        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        node.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        node.create_subscription(Detection2DArray, '/sam_detector/detections',
                                 self._on_detections, 10)
        node.create_subscription(JointTrajectoryControllerState,
                                 '/arm_controller/controller_state', self._on_controller,
                                 qos_profile_sensor_data)
        # Only when a frame arrives is used: a node that must not act on a
        # picture from before some moment (the policy) waits for a newer frame.
        node.create_subscription(CompressedImage, '/image_raw/compressed', self._on_camera,
                                 qos_profile_sensor_data)

        self.target_pub = node.create_publisher(String, '/visual_align/target', LATCHED)
        self.arm_pub = node.create_publisher(JointTrajectory, ARM_TOPIC, 10)
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
        self.last_odom = time.monotonic()
        still = (abs(t.linear.x) < self.still_linear and abs(t.linear.y) < self.still_linear
                 and abs(t.angular.z) < self.still_angular)
        if not still:
            self.still_at = None
        elif self.still_at is None:
            self.still_at = time.monotonic()

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        if len(msg.effort) == len(msg.name):
            self.efforts.update(zip(msg.name, msg.effort))

    def _on_detections(self, msg):
        self.detections_seen += 1
        if msg.detections:
            self.detection_hits += 1
            first = msg.detections[0].results[0].hypothesis if msg.detections[0].results else None
            self.last_hit = (first.class_id, first.score) if first else ('', 0.0)

    def _on_controller(self, msg):
        self.ctrl_reference = dict(zip(msg.joint_names, msg.reference.positions))
        self.ctrl_error = dict(zip(msg.joint_names, msg.error.positions))

    def _on_camera(self, msg):
        self.camera_at = time.monotonic()

    def base_still(self):
        """True while the latest odometry is below the still limits (and is
        still arriving)."""
        return time.monotonic() - self.last_odom < 0.5 and self.still_at is not None

    def settled_at(self):
        """When the robot became settled: the base still and the last arm move
        finished - or None while the base is moving. A camera frame that arrives
        after this shows the robot as it now stands."""
        if self.still_at is None:
            return None
        return max(self.still_at, self.arm_arrived_at)

    def gripper_reads_holding(self):
        """What the gripper joint says right now, by grasp_monitor's own rule
        (mission.yaml grasp_monitor: position above closed_empty_position and
        |effort| above holding_min_effort) - or None until /joint_states has it."""
        joint = self.grasp_cfg.get('joint', GRIPPER_JOINT)
        if joint not in self.joints or joint not in self.efforts:
            return None
        return (self.joints[joint] > float(self.grasp_cfg['closed_empty_position'])
                and abs(self.efforts[joint]) > float(self.grasp_cfg['holding_min_effort']))

    def setting(self, name, given=0.0):
        """A setting from mission.yaml, unless the node was given its own (> 0)."""
        return float(given) if float(given) > 0.0 else float(self.settings[name])

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

    def wait_settled(self, since, timeout_s):
        """RUNNING until the base is still, the arm has finished its last move
        and a camera frame has arrived after both; then SUCCESS. The picture the
        policy starts from must not be from before the robot settled. FAILURE
        after timeout_s (0 = wait for ever)."""
        om = self.om
        settled = om.settled_at()
        if settled is None:
            note = 'waiting for the base to be still'
        elif om.camera_at < settled:
            note = 'waiting for a camera frame after the robot settled'
        else:
            waited = time.monotonic() - since
            self.log_info(f'waited {waited:.1f}s for the base, the arm and a camera frame'
                          if waited >= 0.05 else
                          'base, arm and camera were already settled')
            return SUCCESS
        if timeout_s > 0.0 and time.monotonic() - since > timeout_s:
            return self.fail(f'no camera frame after the robot settled within '
                             f'{timeout_s:g}s ({note}) - is /image_raw/compressed arriving?')
        self.note = note
        return RUNNING

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
