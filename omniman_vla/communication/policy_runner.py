#!/usr/bin/env python3
"""
Runs a policy through Cyclo Intelligence's orchestrator under the control lock
(control_arbiter).

    ~/run     omniman_interfaces/srv/RunPolicy   acquire control, START_INFERENCE
    ~/stop    std_srvs/srv/Trigger               STOP_INFERENCE, release control
    ~/status  std_msgs/msg/String   latched      idle | starting | working
    ~/arm     std_msgs/msg/String   latched      how far the arm is from home,
              and for how long - the signal FINISHED below is made of

A run: acquire control as owner_name ("policy") - refused while someone else
holds it, unless the caller forces - then START_INFERENCE on the orchestrator's
/task/command for `backend` (lerobot), robot mode. The orchestrator loads the
policy if it is not loaded yet (phase LOADING, up to load_timeout_s), then
runs it (INFERENCING); the phases come on /task/inference_status. A run ends
with STOP_INFERENCE, which pauses the policy but keeps it loaded, so the next
run of the same policy starts at once. The call returns once the policy has
been asked to start; follow /control/owner or ~/status to see it end.

FINISHED
    The arm returns to its home pose briefly, as a reset between attempts, and
    for good once the task is done. So finished = at home for finished_dwell_s
    in a row - whether or not it moved first: a policy that stops moving once
    the object is picked leaves the arm home, and that is done. The dwell must
    be longer than any pause at home that is not the end (a reset between
    attempts, a slow start). Arriving home means every arm joint within
    home_tolerance; leaving means some joint beyond it. Keep the arm's resting
    pose well inside home_tolerance, or it flips in and out of home and the
    dwell never completes. Positions only - at rest the reported joint
    velocities are pure noise. On finish: STOP_INFERENCE, then release
    control, which is the "I'm done" whoever is waiting listens for.

A run also ends - and control is given back if still held - when
  - run_timeout_s passes (0 = no limit): the arm never settles at home (it keeps
    moving), so FINISHED above can never happen and the run would hang
  - control is taken away (a forced acquire): STOP_INFERENCE at once
  - inference stops from outside, e.g. in Cyclo's UI
  - the policy fails to load, or is not loaded within load_timeout_s
  - inference never begins within warmup_timeout_s (time spent LOADING
    does not count)
  - ~/stop is called, or the runner shuts down

Nothing here is specific to one task: the caller picks the policy and the
instruction; the home pose and tolerances are parameters (policy_runner.yaml).
The orchestrator must know the robot type (omniman_cyclo's launch sets it).
Recording is never touched - only inference is ever stopped.

Run (with control_arbiter, and its parameters from policy_runner.yaml):
  ros2 launch omniman_vla control_launch.py
"""

import threading
import time

import rclpy
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from interfaces.msg import InferenceStatus
from interfaces.srv import SendCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

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
        self.declare_parameter('backend', 'lerobot')
        self.declare_parameter('inference_hz', 15)
        self.declare_parameter('action_request_mode', 'async')
        self.declare_parameter('control_hz', 100)
        self.declare_parameter('home_pose', [0.0] * len(ARM_JOINTS))
        self.declare_parameter('home_tolerance', 0.20)
        self.declare_parameter('finished_dwell_s', 5.0)
        self.declare_parameter('warmup_timeout_s', 60.0)
        self.declare_parameter('load_timeout_s', 300.0)
        self.declare_parameter('run_timeout_s', 0.0)

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
        # What the finished-check sees: how far the worst arm joint is from
        # home, whether that counts as home, and whether the arm has left it.
        self.arm_pub = self.create_publisher(String, '~/arm', latched)
        self.arm_said = 0.0

        # Orchestrator's inference phase, whoever started it. It is published on
        # changes only (LOADING, INFERENCING, PAUSED, READY), not continuously.
        self.phase = None
        self.phase_error = ''
        self.loading_since = None    # set while the policy is loading
        self.load_spent = 0.0        # loading time of this run so far
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
        self.create_subscription(InferenceStatus, '/task/inference_status',
                                 self.on_inference_status, 10, callback_group=group)
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
        return self.phase == InferenceStatus.INFERENCING

    def call(self, client, request):
        """Call a service and wait for its answer; None on timeout."""
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(CALL_TIMEOUT_S) else None

    def finish(self):
        # STOP_INFERENCE pauses the policy and keeps it loaded (FINISH would
        # unload it, and the next run would wait for a full load again).
        req = SendCommand.Request()
        req.command = SendCommand.Request.STOP_INFERENCE
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
            self.loading_since = None
            self.load_spent = 0.0
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
        start.task_info.service_type = self.get_parameter('backend').value
        start.task_info.inference_mode = 'robot'       # publish to the robot, not a preview
        start.task_info.inference_hz = int(self.get_parameter('inference_hz').value)
        start.task_info.action_request_mode = self.get_parameter('action_request_mode').value
        start.task_info.control_hz = int(self.get_parameter('control_hz').value)
        start.task_info.record_inference_mode = False
        res = self.call(self.command_client, start)
        if res is None or not res.success:
            self.end_run('START refused', finish=False)
            response.success = False
            response.message = (f'START refused: {res.message}' if res
                                else 'orchestrator not answering (/task/command)')
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

    def on_inference_status(self, msg):
        self.phase = msg.inference_phase
        self.phase_error = msg.error
        now = time.monotonic()
        if msg.inference_phase == InferenceStatus.LOADING:
            self.loading_since = self.loading_since or now
        elif self.loading_since is not None:
            self.load_spent += now - self.loading_since
            self.loading_since = None
        if self.state == 'starting' and msg.error:
            self.end_run(f'policy failed to start: {msg.error}', finish=False)
            return
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
            tol = self.get_parameter('home_tolerance').value
            worst = max(abs(positions[j] - h) for j, h in zip(ARM_JOINTS, home))
            now = time.monotonic()
            if now - self.arm_said > 0.5:
                self.arm_said = now
                where = 'at home' if self.at_home else 'away from home'
                since = f', {now - self.home_since:.1f}s' if self.at_home else ''
                self.arm_pub.publish(String(data=(
                    f'{where}{since}: worst joint {worst:.3f} rad (home under '
                    f'{tol}); finished after '
                    f'{self.get_parameter("finished_dwell_s").value}s at home')))

            if self.at_home and worst > tol:
                kind = 'reset' if self.left_home else 'task started'
                self.get_logger().info(
                    f'   left home after {now - self.home_since:.1f}s ({kind})')
                self.at_home = False
                self.left_home = True
            elif not self.at_home and worst <= tol:
                self.at_home = True
                self.home_since = now
                if self.left_home:
                    self.get_logger().info('   back home')
            elif (self.at_home
                  and now - self.home_since >= self.get_parameter('finished_dwell_s').value):
                finished_after = now - self.home_since
        finally:
            self.lock.release()
        if finished_after is not None:
            self.end_run(f'arm finished - home for {finished_after:.1f}s')

    def tick(self):
        now = time.monotonic()
        run_timeout = self.get_parameter('run_timeout_s').value
        loading = now - self.loading_since if self.loading_since is not None else 0.0
        if self.state == 'starting' and loading > self.get_parameter('load_timeout_s').value:
            self.end_run(f'policy not loaded after {loading:.0f}s')
        elif (self.state == 'starting' and not loading
                and now - self.started_at - self.load_spent
                > self.get_parameter('warmup_timeout_s').value):
            self.end_run('inference never started')
        elif (self.state == 'working' and run_timeout > 0.0
                and now - self.started_at > run_timeout):
            self.end_run(f'run timed out after {run_timeout:.0f}s - the arm never stayed '
                         f'home for {self.get_parameter("finished_dwell_s").value}s')
        elif self.state == 'working' and not self.inferencing():
            # Stopped from outside: nothing left to stop.
            self.end_run('inference stopped from outside', finish=False)


def main():
    # rclpy's SIGINT handler would shut the context down before the STOP and
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
