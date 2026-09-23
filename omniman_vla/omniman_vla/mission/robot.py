"""The robot as the steps see it: its topics, its services, and what they
last said. run_mission() makes one Robot and hands it to build(robot)."""

import math
import os
import time

import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from physical_ai_interfaces.msg import TaskStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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

# physical_ai_server's phase numbers, for the log.
TASK_PHASE = {0: 'READY', 1: 'WARMING_UP', 2: 'RESETTING', 3: 'RECORDING', 4: 'SAVING',
              5: 'STOPPED', 6: 'INFERENCING'}


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
        self.policy_status_text = ''
        self.align_status_text = ''
        self.holding = False
        self.grasp_state = 'no reading from grasp_monitor'
        self.arm_state_text = 'no reading from policy_runner'
        self.task_phase_num = -1        # physical_ai_server /task/status
        self.task_phase_at = 0.0
        self.still_since = None
        self.last_odom = 0.0
        self.pose = (0.0, 0.0, 0.0)     # x, y, yaw from odometry
        self.twist = (0.0, 0.0, 0.0)    # latest vx, vy, wz from odometry

        nav.create_subscription(ControlOwner, '/control/owner',
                                lambda m: setattr(self, 'owner', m.owner), LATCHED)
        nav.create_subscription(String, '/policy_runner/status',
                                lambda m: setattr(self, 'policy_status_text', m.data), LATCHED)
        nav.create_subscription(String, '/policy_runner/arm',
                                lambda m: setattr(self, 'arm_state_text', m.data), LATCHED)
        nav.create_subscription(String, '/visual_align/status',
                                lambda m: setattr(self, 'align_status_text', m.data), LATCHED)
        nav.create_subscription(Bool, '/gripper/holding',
                                lambda m: setattr(self, 'holding', m.data), LATCHED)
        nav.create_subscription(String, '/grasp_monitor/state',
                                lambda m: setattr(self, 'grasp_state', m.data), LATCHED)
        nav.create_subscription(TaskStatus, '/task/status', self._on_task_status, 10)
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

    def _on_task_status(self, msg):
        self.task_phase_num, self.task_phase_at = msg.phase, time.monotonic()

    def task_phase_text(self):
        """What physical_ai_server is doing, or how long it has said nothing
        (it publishes /task/status on every inference tick and goes quiet in
        between runs)."""
        if self.task_phase_num < 0:
            return '/task/status: nothing yet'
        quiet = time.monotonic() - self.task_phase_at
        name = TASK_PHASE.get(self.task_phase_num, self.task_phase_num)
        return (f'/task/status: {name}' if quiet < 1.0
                else f'/task/status: {name} (quiet for {quiet:.0f}s)')

    # ---- what the robot knows, for conditions and for logging ------------

    def is_holding(self):
        """True while grasp_monitor reports something in the gripper."""
        return self.holding

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
        """physical_ai_server's phase, e.g. "INFERENCING" or "READY"."""
        return self.task_phase_text()

    def control_owner(self):
        """Who holds the control lock: "nav" | "align" | "policy" | "" ."""
        return self.owner

    def base_pose(self):
        """(x, y, yaw) from wheel odometry."""
        return self.pose

    def base_twist(self):
        """(vx, vy, wz) from wheel odometry."""
        return self.twist

    def base_still(self):
        """True once odometry has shown the base stopped for still_time_s."""
        now = time.monotonic()
        return (now - self.last_odom < 0.5 and self.still_since is not None
                and now - self.still_since >= float(self.settings['still_time_s']))

    def release(self):
        req = ReleaseControl.Request()
        req.owner = NAV
        self.release_client.call_async(req)
