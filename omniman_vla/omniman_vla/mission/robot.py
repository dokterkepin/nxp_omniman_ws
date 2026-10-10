"""The robot as the steps see it: its topics, its services, and what they
last said. run_mission() makes one Robot and hands it to build(robot)."""

import math
import os
import time

import yaml
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from interfaces.msg import InferenceStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from vision_msgs.msg import Detection2DArray

# Not settings - part of the contract with other nodes:
#  - the owner name for driving: control_arbiter's other users (the web UI's
#    lock display, other missions) know this robot's driver as "nav".
NAV = 'nav'
#  - the owner name for moving the arm to a pose (ArmPose).
ARM = 'arm'
#  - the QoS the latched topics (/control/owner, the statuses, the gripper
#    state) are published with; a subscriber must use the same to get them.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
#  - the same, but a subscriber keeps up to 10 messages between two spins: the
#    steps wait for a status to go busy and then done, and a status that
#    changes twice in one tick must not lose the busy one.
STATUS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

#  - the arm controller's joints, in its order (controllers_vla.yaml). A
#    trajectory must name all of them: the controller refuses fewer.
ARM_JOINTS = ['shoulder_yaw_joint', 'upper_shoulder_pitch_joint', 'arm_yaw_joint',
              'forearm_pitch_joint', 'wrist_pitch_joint', 'palm_yaw_joint']
GRIPPER_JOINT = 'gripper_prismatic_joint'
#  - where arm moves go: the topic the policy publishes on; the teleop relay
#    (teleop_bridges_launch.py) passes it to /arm_controller/joint_trajectory.
ARM_TOPIC = '/leader/joint_trajectory'

# Cyclo orchestrator's inference phases (interfaces/msg/InferenceStatus), for the log.
TASK_PHASE = {0: 'READY', 1: 'LOADING', 2: 'INFERENCING', 3: 'PAUSED', 4: 'SYNCING'}


# ---- places ---------------------------------------------------------------

def load_poses(mission_file):
    """Places from poses.yaml beside the mission file (saved from the web UI)."""
    path = os.path.join(os.path.dirname(os.path.realpath(mission_file)), 'poses.yaml')
    with open(path) as f:
        return yaml.safe_load(f) or {}


def load_arm_poses(mission_file):
    """Arm poses from arm_poses.yaml beside the mission file ({} if there is none)."""
    path = os.path.join(os.path.dirname(os.path.realpath(mission_file)), 'arm_poses.yaml')
    if not os.path.exists(path):
        return {}
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
        self.policy_status_text = ''
        self.align_status_text = ''
        self.holding = False
        self.grasp_state = 'no reading from grasp_monitor'
        self.arm_state_text = 'no reading from policy_runner'
        self.arm_state_at = 0.0         # when it arrived (monotonic)
        self.task_phase_num = -1        # orchestrator /task/inference_status
        self.task_phase_at = 0.0
        self.still_at = None            # when odometry first showed the base still, this stop
        self.last_odom = 0.0
        self.pose = (0.0, 0.0, 0.0)     # x, y, yaw from odometry
        self.twist = (0.0, 0.0, 0.0)    # latest vx, vy, wz from odometry
        self.joints = {}                # joint name -> position, from /joint_states
        self.efforts = {}               # joint name -> effort, from /joint_states
        self.detections_seen = 0        # detector messages so far (empty ones too)
        self.detection_hits = 0         # ... of which had something in them
        self.last_hit = ('', 0.0)       # what the latest hit was: (prompt, score)
        # Times below are when the messages ARRIVED here, on this node's clock - not
        # the messages' own stamps: the camera's header stamps are about 0.6 s early
        # (measured: the picture reacts 0.04 s after the joints, its stamp says -0.6 s).
        self.arm_arrived_at = 0.0       # when the arm controller said a move had finished
        self.camera_at = 0.0            # when the latest camera frame arrived
        self.ctrl_reference = {}        # arm_controller: the trajectory point it is tracking
        self.ctrl_error = {}            # arm_controller: tracking error per joint

        nav.create_subscription(ControlOwner, '/control/owner',
                                lambda m: setattr(self, 'owner', m.owner), LATCHED)
        nav.create_subscription(String, '/policy_runner/status',
                                lambda m: setattr(self, 'policy_status_text', m.data), STATUS)
        nav.create_subscription(String, '/policy_runner/arm', self._on_arm_state, LATCHED)
        nav.create_subscription(String, '/visual_align/status',
                                lambda m: setattr(self, 'align_status_text', m.data), STATUS)
        nav.create_subscription(Bool, '/gripper/holding',
                                lambda m: setattr(self, 'holding', m.data), LATCHED)
        nav.create_subscription(String, '/grasp_monitor/state',
                                lambda m: setattr(self, 'grasp_state', m.data), LATCHED)
        nav.create_subscription(InferenceStatus, '/task/inference_status',
                                self._on_task_status, 10)
        nav.create_subscription(Odometry, '/mecanum_drive_controller/odometry',
                                self._on_odom, 10)
        nav.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        nav.create_subscription(Detection2DArray, '/sam_detector/detections',
                                self._on_detections, 10)
        # The arm controller's own report: where its trajectory is and how far the
        # arm is from it - the arm has finished a move when the trajectory has
        # reached its end and the error is small.
        nav.create_subscription(JointTrajectoryControllerState,
                                '/arm_controller/controller_state', self._on_controller,
                                qos_profile_sensor_data)
        # Only when a frame arrives is used: a step that must not act on a
        # picture older than some moment (the policy) waits for a newer frame.
        nav.create_subscription(CompressedImage, '/image_raw/compressed', self._on_camera,
                                qos_profile_sensor_data)
        self.target_pub = nav.create_publisher(String, '/visual_align/target', LATCHED)
        self.arm_pub = nav.create_publisher(JointTrajectory, ARM_TOPIC, 10)

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
        self.last_odom = time.monotonic()
        self.twist = (t.linear.x, t.linear.y, t.angular.z)
        lin, ang = float(self.settings['still_linear']), float(self.settings['still_angular'])
        moving = abs(t.linear.x) >= lin or abs(t.linear.y) >= lin or abs(t.angular.z) >= ang
        if moving:
            self.still_at = None
        elif self.still_at is None:
            self.still_at = self.now()

    def _on_arm_state(self, msg):
        self.arm_state_text, self.arm_state_at = msg.data, time.monotonic()

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        if len(msg.effort) == len(msg.name):
            self.efforts.update(zip(msg.name, msg.effort))

    def _on_detections(self, msg):
        self.detections_seen += 1
        if msg.detections:
            self.detection_hits += 1
            result = msg.detections[0].results[0].hypothesis if msg.detections[0].results else None
            self.last_hit = (result.class_id, result.score) if result else ('', 0.0)

    def _on_controller(self, msg):
        self.ctrl_reference = dict(zip(msg.joint_names, msg.reference.positions))
        self.ctrl_error = dict(zip(msg.joint_names, msg.error.positions))

    def _on_camera(self, msg):
        self.camera_at = self.now()

    def now(self):
        """This node's ROS time, in seconds."""
        return self.nav.get_clock().now().nanoseconds / 1e9

    def _on_task_status(self, msg):
        self.task_phase_num, self.task_phase_at = msg.inference_phase, time.monotonic()

    def task_phase_text(self):
        """What the orchestrator's policy is doing, and since when (it
        announces each change of phase, not a continuous stream)."""
        if self.task_phase_num < 0:
            return '/task/inference_status: nothing yet'
        since = time.monotonic() - self.task_phase_at
        name = TASK_PHASE.get(self.task_phase_num, self.task_phase_num)
        return f'/task/inference_status: {name} for {since:.0f}s'

    # ---- what the robot knows, for conditions and for logging ------------

    def is_holding(self):
        """True while grasp_monitor reports something in the gripper."""
        return self.holding

    def gripper_reads_holding(self):
        """What the gripper joint says right now, by grasp_monitor's own rule
        (mission.yaml grasp_monitor: position above closed_empty_position and
        |effort| above holding_min_effort) - or None until /joint_states has it."""
        cfg = self.cfg['grasp_monitor']
        joint = cfg['joint']
        if joint not in self.joints or joint not in self.efforts:
            return None
        return (self.joints[joint] > float(cfg['closed_empty_position'])
                and abs(self.efforts[joint]) > float(cfg['holding_min_effort']))

    def gripper_state(self):
        """grasp_monitor's reading, e.g. "holding (position +0.0008, effort -103)"."""
        return self.grasp_state

    def arm_state(self):
        """policy_runner's view of the arm: how far from home, and for how long."""
        return self.arm_state_text

    def align_status(self):
        """visual_align: searching | aligning | aligned | failed: <why> | idle."""
        return self.align_status_text

    def policy_status(self):
        """policy_runner: idle | starting | working | ending."""
        return self.policy_status_text

    def task_phase(self):
        """The orchestrator's policy phase, e.g. "INFERENCING" or "READY"."""
        return self.task_phase_text()

    def control_owner(self):
        """Who holds the control lock: "nav" | "align" | "policy" | "" ."""
        return self.owner

    def arm_joints(self):
        """{joint name: position} of every joint seen on /joint_states."""
        return dict(self.joints)

    def base_pose(self):
        """(x, y, yaw) from wheel odometry."""
        return self.pose

    def base_twist(self):
        """(vx, vy, wz) from wheel odometry."""
        return self.twist

    def base_still(self):
        """True while the latest odometry is below the still_linear / still_angular
        limits (and odometry is still arriving)."""
        return time.monotonic() - self.last_odom < 0.5 and self.still_at is not None

    def settled_at(self):
        """When the robot became settled: the base still and the last arm move
        finished - or None while the base is moving. A camera frame that arrives
        after this shows the robot as it now stands (the camera's own delay is
        about 0.04 s)."""
        if self.still_at is None:
            return None
        return max(self.still_at, self.arm_arrived_at)

    def release(self, owner=NAV):
        req = ReleaseControl.Request()
        req.owner = owner
        self.release_client.call_async(req)
