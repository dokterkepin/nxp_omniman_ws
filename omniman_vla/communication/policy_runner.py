#!/usr/bin/env python3
"""
Runs a physical_ai_server policy under the control lock (control_arbiter).

    ~/run     omniman_interfaces/srv/RunPolicy   acquire control, START_INFERENCE
    ~/stop    std_srvs/srv/Trigger               FINISH, release control
    ~/status  std_msgs/msg/String   latched      idle | starting | working

A run: acquire control as owner_name ("policy") - refused while someone else
holds it, unless the caller forces - then START_INFERENCE. The call returns
once the policy has started; follow /control/owner or ~/status to see it end.

FINISHED
    The arm returns to its home pose briefly, as a reset between attempts, and
    for good once the task is done. So finished = back home, after having left
    it once, for finished_dwell_s. Arriving home means every arm joint within
    home_tolerance; leaving means some joint beyond home_exit_tolerance, so a
    joint resting near the edge cannot flap. Positions only - at rest the
    reported joint velocities are pure noise. On finish: FINISH, then release
    control, which is the "I'm done" whoever is waiting listens for.

A run also ends - and control is given back if still held - when
  - control is taken away (a forced acquire): FINISH at once
  - inference stops from outside, e.g. in the physical_ai_manager UI
  - inference never begins within warmup_timeout_s
  - ~/stop is called, or the runner shuts down

Nothing here is specific to one task: the caller picks the policy and the
instruction; the home pose and tolerances are parameters (policy_runner.yaml).
RECORDING is never touched - only phase INFERENCING is ever finished.

Run (with control_arbiter, and its parameters from policy_runner.yaml):
  ros2 launch omniman_vla control_launch.py
"""

import threading
import time

import rclpy
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import SendCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

# physical_ai_server publishes /task/status on every inference tick (~30 Hz)
# and goes quiet once inference ends, so an old INFERENCING is not trusted.
STATUS_STALE_S = 1.0

CALL_TIMEOUT_S = 15.0

# Arm joints checked against home_pose, in that order. The gripper is left out
# on purpose - different tasks end it open or closed.
ARM_JOINTS = [
    'shoulder_yaw_joint',
    'upper_shoulder_pitch_joint',
    'arm_yaw_joint',
    'forearm_pitch_joint',
    'wrist_pitch_joint',
    'palm_yaw_joint',
]


class PolicyRunner(Node):

    def __init__(self):
        super().__init__('policy_runner')
        self.declare_parameter('owner_name', 'policy')
        self.declare_parameter('policy_path', '')
        self.declare_parameter('instruction', '')
        self.declare_parameter('fps', 30)
        self.declare_parameter('home_pose', [0.0] * len(ARM_JOINTS))
        self.declare_parameter('home_tolerance', 0.20)
        self.declare_parameter('home_exit_tolerance', 0.30)
        self.declare_parameter('finished_dwell_s', 5.0)
        self.declare_parameter('warmup_timeout_s', 60.0)

        # Service handlers call other services and wait for the answer, which
        # needs other threads spinning - hence reentrant + multithreaded.
        # The lock guards state only and is NEVER held while waiting on a
        # service: /joint_states arrives at 100 Hz, and callbacks queued on a
        # lock held across a call would take every executor thread, leaving
        # none to deliver the reply.
        group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        self.acquire_client = self.create_client(
            AcquireControl, '/control/acquire', callback_group=group)
        self.release_client = self.create_client(
            ReleaseControl, '/control/release', callback_group=group)
        self.command_client = self.create_client(
            SendCommand, '/task/command', callback_group=group)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', latched)

        # Server phase, whoever started it.
        self.phase = None
        self.phase_time = 0.0
        self.owner = ''

        # idle -> starting -> working (server inferencing) -> ending -> idle
        self.state = ''
        # True from a successful acquire until the run ends. Only while it is
        # set does a change of /control/owner mean "control was taken away".
        self.holding = False
        self.started_at = 0.0
        self.at_home = False
        self.home_since = 0.0
        self.left_home = False

        self.create_subscription(
            ControlOwner, '/control/owner', self.on_owner, latched, callback_group=group)
        self.create_subscription(
            TaskStatus, '/task/status', self.on_task_status, 10, callback_group=group)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10, callback_group=group)
        self.create_service(RunPolicy, '~/run', self.on_run, callback_group=group)
        self.create_service(Trigger, '~/stop', self.on_stop, callback_group=group)
        self.create_timer(0.2, self.tick, callback_group=group)

        self.set_state('idle')
        self.get_logger().info(f'ready - runs policies as owner "{self.owner_name()}"')

    # ---- helpers ------------------------------------------------------------

    def owner_name(self):
        return self.get_parameter('owner_name').value

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.status_pub.publish(String(data=state))

    def inferencing(self):
        return (self.phase == TaskStatus.INFERENCING
                and time.monotonic() - self.phase_time < STATUS_STALE_S)

    def call(self, client, request):
        """Call a service and wait for its answer; None on timeout."""
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(CALL_TIMEOUT_S) else None

    def finish(self):
        # FINISH, not STOP: only FINISH clears the server's on_inference flag.
        req = SendCommand.Request()
        req.command = SendCommand.Request.FINISH
        self.call(self.command_client, req)

    def release(self):
        req = ReleaseControl.Request()
        req.owner = self.owner_name()
        self.call(self.release_client, req)

    def end_run(self, why, finish=True, release=True):
        """Every way a run ends goes through here, exactly once per run."""
        with self.lock:
            if self.state not in ('starting', 'working'):
                return
            self.set_state('ending')
            holding, self.holding = self.holding, False
        self.get_logger().info(f'{why} - ending run')
        if finish:
            self.finish()
        if release and holding:
            self.release()
        self.set_state('idle')

    # ---- services -----------------------------------------------------------

    def on_run(self, request, response):
        path = request.policy_path or self.get_parameter('policy_path').value
        instruction = request.instruction or self.get_parameter('instruction').value
        self.get_logger().info(f'run requested: "{instruction}" ({path})')

        if not path:
            response.success = False
            response.message = 'no policy_path given and no default set'
            return response

        # Reserve the runner, so a second run is refused while this one starts.
        with self.lock:
            if self.state != 'idle':
                response.success = False
                response.message = f'already {self.state}'
                return response
            self.set_state('starting')
            self.started_at = time.monotonic()
            self.at_home = False
            self.left_home = False

        acquire = AcquireControl.Request()
        acquire.owner = self.owner_name()
        acquire.node = self.get_fully_qualified_name()
        acquire.force = request.force
        got = self.call(self.acquire_client, acquire)
        if got is None or not got.success:
            self.set_state('idle')
            response.success = False
            response.message = got.message if got else 'control_arbiter not answering'
            return response
        # Only watch for takeovers once /control/owner shows us as owner: an
        # older "nobody" message still on its way would otherwise read as
        # control being taken the moment we got it.
        deadline = time.monotonic() + 3.0
        while self.owner != self.owner_name() and time.monotonic() < deadline:
            time.sleep(0.02)
        with self.lock:
            self.holding = True

        start = SendCommand.Request()
        start.command = SendCommand.Request.START_INFERENCE
        start.task_info.policy_path = path
        start.task_info.task_instruction = [instruction]
        start.task_info.fps = int(self.get_parameter('fps').value)
        start.task_info.record_inference_mode = False
        res = self.call(self.command_client, start)
        if res is None or not res.success:
            self.end_run('START refused', finish=False)
            response.success = False
            response.message = (f'START refused: {res.message}' if res
                                else 'physical_ai_server not answering')
            return response

        with self.lock:
            # Control may have been taken while START was on its way.
            ended = self.state != 'starting'
        if ended:
            self.finish()
            response.success = False
            response.message = 'control was taken while starting'
            return response

        self.get_logger().info(f'started "{instruction}" ({path})')
        response.success = True
        response.message = f'policy started: "{instruction}"'
        return response

    def on_stop(self, request, response):
        if self.state in ('starting', 'working'):
            self.end_run('stop requested')
            response.message = 'stopped'
        elif self.inferencing() and self.owner == self.owner_name():
            # A run this node lost track of (e.g. restarted): still clean up.
            self.finish()
            self.release()
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
            # Control is already gone, so there is nothing to release.
            self.end_run(f'control taken by {taker}', release=False)

    def on_task_status(self, msg):
        self.phase = msg.phase
        self.phase_time = time.monotonic()
        if self.state == 'starting' and self.inferencing():
            with self.lock:
                if self.state != 'starting':
                    return
                self.set_state('working')
            self.get_logger().info('inference running - watching for the arm to finish')

    def on_joint_states(self, msg):
        if self.state != 'working':
            return
        # Skip this sample rather than queue behind the lock - another one
        # arrives in 10 ms.
        if not self.lock.acquire(blocking=False):
            return
        finished_after = None
        try:
            positions = dict(zip(msg.name, msg.position))
            if any(j not in positions for j in ARM_JOINTS):
                return
            home = self.get_parameter('home_pose').value
            enter_tol = self.get_parameter('home_tolerance').value
            exit_tol = self.get_parameter('home_exit_tolerance').value
            worst = max(abs(positions[j] - h) for j, h in zip(ARM_JOINTS, home))
            now = time.monotonic()

            if self.at_home and worst > exit_tol:
                kind = 'reset' if self.left_home else 'task started'
                self.get_logger().info(
                    f'   left home after {now - self.home_since:.1f}s ({kind})')
                self.at_home = False
                self.left_home = True
            elif not self.at_home and worst <= enter_tol:
                self.at_home = True
                self.home_since = now
                if self.left_home:
                    self.get_logger().info('   back home')
            elif (self.at_home and self.left_home
                  and now - self.home_since >= self.get_parameter('finished_dwell_s').value):
                finished_after = now - self.home_since
        finally:
            self.lock.release()
        if finished_after is not None:
            self.end_run(f'arm finished - home for {finished_after:.1f}s')

    def tick(self):
        now = time.monotonic()
        if (self.state == 'starting'
                and now - self.started_at > self.get_parameter('warmup_timeout_s').value):
            self.end_run('inference never started')
        elif self.state == 'working' and not self.inferencing():
            # Stopped from outside: nothing left to FINISH.
            self.end_run('inference stopped from outside', finish=False)


def main():
    # rclpy's SIGINT handler would shut the context down before the FINISH and
    # release below could be sent; Python's default just raises KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PolicyRunner()
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
        node.end_run('shutting down')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
