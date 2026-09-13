#!/usr/bin/env python3
"""
Start an arm policy as soon as a Nav2 goal is reached - no fixed delays.

    Nav2 goal SUCCEEDED -> wait until the base is still -> START_INFERENCE
    new Nav2 goal       -> FINISH (the arm must not work while the base drives)

Any goal counts, e.g. Foxglove's goal pose tool or RViz's 2D Goal Pose.
"Still" means wheel odometry under STILL_LINEAR / STILL_ANGULAR for
still_time_s. This replaces the fixed settle_s sleep in pick_place_shuttle.py.

It is a listener, not an owner: /task/status says whether physical_ai_server
is inferencing, so a policy started anywhere - here, pick_place_shuttle.py, or
the physical_ai_manager UI - is stopped the moment a Nav2 goal starts. Without
that, Nav2 and the policy both write /cmd_vel and the base jitters.

RECORDING is never touched. FINISH would end a recording session too, so the
phase from /task/status is what makes stopping safe: only phase INFERENCING is
ever finished.

The policy runs until you call ~/stop, stop it in the UI, or send a new goal.

HOME VISITS (measuring, not acting yet)
    While inferencing, the node logs every visit of the arm to its home pose
    and how long it stayed. A reset is a short visit before the arm moves again;
    a finished task is a long one. Run a few real attempts, read the
    'arm left home after Xs' lines, and set finished_dwell_s above the longest
    reset. Nothing is stopped or blocked on this yet.

Prereqs:
  - nav2_launch.py
  - physical_ai_server_bringup.launch.py (serves /task/command)

Run:
  ros2 run omniman_navigation policy_on_arrival.py --ros-args \
      -p policy_path:=/path/to/pretrained_model -p instruction:="pick the object"

Stop:
  ros2 service call /policy_on_arrival/stop std_srvs/srv/Trigger
"""

import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import SendCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

STILL_LINEAR = 0.01     # m/s
STILL_ANGULAR = 0.02    # rad/s
SETTLE_TIMEOUT_S = 5.0  # give up if the base has not stopped by then

# The arm's home pose: where every policy episode starts, resets between
# attempts, and ends. Measured from /joint_states at rest (mean over 6 s).
# The gripper is left out on purpose - different tasks end it open or closed.
ARM_JOINTS = [
    'shoulder_yaw_joint',
    'upper_shoulder_pitch_joint',
    'arm_yaw_joint',
    'forearm_pitch_joint',
    'wrist_pitch_joint',
    'palm_yaw_joint',
]
HOME_POSE = [0.077, 0.573, -0.079, -1.517, 1.436, -0.054]

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)


class PolicyOnArrival(Node):

    def __init__(self):
        super().__init__('policy_on_arrival')
        self.declare_parameter(
            'policy_path',
            '/home/dokterkepin/output/omniman_pick_and_place/'
            'checkpoints/last/pretrained_model')
        self.declare_parameter('instruction', 'pick the object')
        self.declare_parameter('fps', 30)
        self.declare_parameter('still_time_s', 0.5)
        # false: only act as the arbiter that stops the policy on a new goal,
        # e.g. while pick_place_shuttle.py runs the mission itself.
        self.declare_parameter('auto_start', True)
        # Home detection. At rest the arm still jitters up to ~0.03 rad
        # peak-to-peak, and its reported velocities spike to ~0.5 rad/s, so
        # this is position-only: inside home_tolerance of HOME_POSE on every
        # arm joint. 0.08 rad is ~3x the worst jitter.
        self.declare_parameter('home_pose', HOME_POSE)
        self.declare_parameter('home_tolerance', 0.08)
        # A reset also passes through home, but only briefly before the arm
        # moves again; a finished task stays there. So "finished" is a dwell
        # longer than any reset pause. Not tuned yet - read the home-visit log
        # lines from real runs and set this above the longest reset.
        self.declare_parameter('finished_dwell_s', 5.0)

        self.client = self.create_client(SendCommand, '/task/command')

        # Phase from /task/status: what physical_ai_server is doing right now,
        # whoever asked it to. None until the first message arrives.
        self.phase = None

        # Home visits are only timed while inferencing: during WARMING_UP the
        # arm sits at home while the model loads, and that must not count.
        self.home_since = None          # monotonic time the arm reached home
        self.finished_reported = False

        # idle -> settling (goal reached) -> starting (START sent) -> running
        self.state = 'idle'
        self.goal_status = {}       # goal uuid -> last status seen
        self.settle_started = None
        self.still_since = None     # None while the base moves
        self.last_odom = None

        # Action status topics are reliable + transient_local.
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_nav_status, status_qos)
        self.create_subscription(
            Odometry, '/mecanum_drive_controller/odometry', self.on_odom, 10)
        self.create_subscription(
            TaskStatus, '/task/status', self.on_task_status, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'ready - waiting for a Nav2 goal'
            + ('' if self.get_parameter('auto_start').value
               else ' (auto_start off: stop-only)'))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        t = msg.twist.twist
        still = (abs(t.linear.x) < STILL_LINEAR
                 and abs(t.linear.y) < STILL_LINEAR
                 and abs(t.angular.z) < STILL_ANGULAR)
        self.last_odom = self.now()
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = self.last_odom

    def inferencing(self):
        return self.phase == TaskStatus.INFERENCING

    # ---- home detection ---------------------------------------------------

    def on_task_status(self, msg):
        was = self.inferencing()
        self.phase = msg.phase
        now_inferencing = self.inferencing()
        if now_inferencing and not was:
            self.get_logger().info('inference started - timing home visits')
            self.home_since = None
        elif was and not now_inferencing:
            if self.home_since is not None:
                self.get_logger().info(
                    f'inference ended while home for '
                    f'{time.monotonic() - self.home_since:.1f}s')
            self.home_since = None

    def on_joint_states(self, msg):
        if not self.inferencing():
            return
        positions = dict(zip(msg.name, msg.position))
        if any(j not in positions for j in ARM_JOINTS):
            return

        home = self.get_parameter('home_pose').value
        tol = self.get_parameter('home_tolerance').value
        errors = [abs(positions[j] - h) for j, h in zip(ARM_JOINTS, home)]
        at_home = max(errors) <= tol
        now = time.monotonic()

        since = self.home_since
        if at_home and since is None:
            self.home_since = now
            self.finished_reported = False
            self.get_logger().info('arm reached home')

        elif at_home and since is not None:
            dwell = now - since
            if (not self.finished_reported
                    and dwell >= self.get_parameter('finished_dwell_s').value):
                self.finished_reported = True
                self.get_logger().info(f'arm FINISHED - home for {dwell:.1f}s')

        elif since is not None:
            dwell = now - since
            worst = ARM_JOINTS[errors.index(max(errors))]
            note = (' - longer than finished_dwell_s, would have ended the task'
                    if self.finished_reported else '')
            self.get_logger().info(
                f'arm left home after {dwell:.1f}s '
                f'({worst} moved {max(errors):.3f} rad){note}')
            self.home_since = None

    def on_nav_status(self, msg):
        previous = self.goal_status
        # The list holds every goal the server still remembers, so rebuilding
        # it from each message also forgets expired goals.
        self.goal_status = {
            bytes(s.goal_info.goal_id.uuid): s.status for s in msg.status_list}

        for gid, status in self.goal_status.items():
            before = previous.get(gid)
            if status in ACTIVE and before not in ACTIVE:
                self.stop('new Nav2 goal')
            # Goals that finished before this node started arrive with
            # before=None (transient_local replay), so they never trigger.
            # Preempted goals end ABORTED and cancelled ones CANCELED.
            elif status == GoalStatus.STATUS_SUCCEEDED and before in ACTIVE:
                if not self.get_parameter('auto_start').value:
                    continue
                # pick_place_shuttle.py starts its own policy after arriving;
                # two starts would fight, so stand back if anything is running.
                if self.phase not in (None, TaskStatus.READY, TaskStatus.STOPPED):
                    self.get_logger().info(
                        f'goal reached - not starting, server is busy '
                        f'(phase {self.phase})')
                    continue
                self.state = 'settling'
                self.settle_started = self.now()
                self.get_logger().info('goal reached - waiting for the base to stop')

    def on_stop(self, request, response):
        self.stop('stop requested')
        response.success = True
        response.message = 'stopped'
        return response

    def tick(self):
        if self.state != 'settling':
            return
        now = self.now()
        odom_fresh = self.last_odom is not None and now - self.last_odom < 0.5
        still_for = now - self.still_since if self.still_since else 0.0
        if odom_fresh and still_for >= self.get_parameter('still_time_s').value:
            self.start_policy()
        elif now - self.settle_started > SETTLE_TIMEOUT_S:
            why = 'still moving' if odom_fresh else 'no odometry'
            self.get_logger().error(f'base did not settle ({why}) - policy not started')
            self.state = 'idle'

    def start_policy(self):
        if not self.client.service_is_ready():
            self.get_logger().error('/task/command unavailable - physical_ai_server running?')
            self.state = 'idle'
            return

        req = SendCommand.Request()
        req.command = SendCommand.Request.START_INFERENCE
        req.task_info.policy_path = self.get_parameter('policy_path').value
        req.task_info.task_instruction = [self.get_parameter('instruction').value]
        req.task_info.fps = self.get_parameter('fps').value
        req.task_info.record_inference_mode = False

        self.state = 'starting'
        self.get_logger().info(f'starting policy "{req.task_info.task_instruction[0]}"')
        self.client.call_async(req).add_done_callback(self.on_started)

    def on_started(self, future):
        res = future.result()
        if self.state != 'starting':
            # Stopped while START was in flight - undo it.
            self.send_finish()
        elif res is None or not res.success:
            self.get_logger().error(f'policy start refused: {res.message if res else "no reply"}')
            self.state = 'idle'
        else:
            self.state = 'running'
            self.get_logger().info('policy running')

    def stop(self, why):
        # Stop a policy no matter who started it, but never a recording: the
        # phase from /task/status is what tells those apart.
        mine = self.state in ('starting', 'running')
        if mine or self.inferencing():
            self.get_logger().info(f'{why} - stopping policy')
            self.send_finish()
        elif self.phase is not None and self.phase != TaskStatus.READY:
            self.get_logger().info(
                f'{why} - leaving physical_ai_server alone (phase {self.phase})')
        self.state = 'idle'

    def send_finish(self):
        # FINISH, not STOP: only FINISH clears the server's on_inference flag.
        req = SendCommand.Request()
        req.command = SendCommand.Request.FINISH
        return self.client.call_async(req)


def main():
    # rclpy's own SIGINT handler shuts the context down, and the FINISH below
    # could then not be sent. Python's default one just raises
    # KeyboardInterrupt, which the 20 Hz timer lets spin notice.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PolicyOnArrival()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state in ('starting', 'running'):
            node.get_logger().info('shutting down - stopping policy')
            rclpy.spin_until_future_complete(node, node.send_finish(), timeout_sec=3.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
