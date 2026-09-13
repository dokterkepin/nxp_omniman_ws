#!/usr/bin/env python3
"""
Pick-and-place shuttle with nav2 + one ACT policy, no correction step.

Same mission as pick_place_shuttle.py, minus the base_correction policy: nav2
drives, then the manipulate policy runs straight away. Use this when no
base_correction checkpoint is trained, or to see how much the policy can
absorb nav2's residual error on its own.

    home -> pick_area   nav2
            pick        manipulate policy
    ->      place_area  nav2
            place       manipulate policy (same checkpoint)
    -> home             nav2

Everything tunable lives in config/mission.yaml - poses, policy paths,
instructions, durations. Nothing here needs editing to change the mission.

KNOWING WHEN A POLICY IS DONE
    physical_ai_server has no "task finished" signal, so each policy runs until
    the arm returns to its home pose and stays there for finished_dwell_s
    (TaskFinished). A reset between grasp attempts also passes through home,
    but only briefly. There is no time limit - the checker alone decides.

WAITING, NOT SLEEPING
    After each nav leg the mission waits for wheel odometry to show the base
    has actually stopped (BaseStill), rather than sleeping a fixed settle_s.
    policy_on_arrival.py does the same for goals sent by hand from the iPad.

NAV2'S RESIDUAL ERROR
    Nav2 stops when SimpleGoalChecker is satisfied, which on this robot means up
    to ~5.7 deg and ~12 cm of residual error - and it cannot do better, because
    AMCL only knows the yaw to ~4.7 deg (1 sigma). The manipulate policy has to
    absorb that much variation in where the base ends up. A base_correction
    policy used to run first to close the gap visually; it is not used now.

MUTUAL EXCLUSION
    Nav2 drives the base over /cmd_vel; the policies drive the base over the SAME
    topic (leader_mobile publishes Twist) plus the arm over /leader/joint_trajectory.
    They must never run together. Actuator guarantees that: every acquire releases
    the other first, and every exit path releases both.

Prereqs:
  - nav2_launch.py, robot localized
  - physical_ai_server_bringup.launch.py (serves /task/command)

Run:
  ros2 run omniman_navigation pick_place_simple.py
  ros2 run omniman_navigation pick_place_simple.py --ros-args \
      -p mission_file:=/path/to/mission.yaml
"""

import math
import time
from enum import Enum, auto

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import SendCommand
from sensor_msgs.msg import JointState

# "Still" means every odometry twist component under these for still_time_s.
# Same test as policy_on_arrival.py, which does this for hand-sent goals.
STILL_LINEAR = 0.01     # m/s
STILL_ANGULAR = 0.02    # rad/s

# Arm joints checked against the home pose. The gripper is left out on
# purpose - different tasks end it open or closed. The pose itself lives in
# mission.yaml (settings.home_pose), in this order.
ARM_JOINTS = [
    'shoulder_yaw_joint',
    'upper_shoulder_pitch_joint',
    'arm_yaw_joint',
    'forearm_pitch_joint',
    'wrist_pitch_joint',
    'palm_yaw_joint',
]

# physical_ai_server publishes /task/status on every inference tick (~30 Hz)
# and goes quiet once inference ends, so an old INFERENCING is not trusted.
STATUS_STALE_S = 1.0


class State(Enum):
    NAV_TO_PICK = auto()
    PICK = auto()
    NAV_TO_PLACE = auto()
    PLACE = auto()
    NAV_TO_HOME = auto()
    DONE = auto()
    ABORT = auto()


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


class BaseStill:
    """Waits until wheel odometry says the base really stopped.

    Nav2 reports success as soon as the goal checker is satisfied, while the
    base is still coasting. A fixed sleep afterwards is always wrong: too long
    and the mission crawls, too short and the policy starts from a pose that is
    already stale. This waits for the actual stop instead.
    """

    def __init__(self, node):
        self.node = node
        self.still_since = None
        self.last_odom = None
        node.create_subscription(
            Odometry, '/mecanum_drive_controller/odometry', self._on_odom, 10)

    def _on_odom(self, msg):
        t = msg.twist.twist
        now = time.monotonic()
        self.last_odom = now
        moving = (abs(t.linear.x) >= STILL_LINEAR
                  or abs(t.linear.y) >= STILL_LINEAR
                  or abs(t.angular.z) >= STILL_ANGULAR)
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def wait(self, still_time_s, timeout_s):
        """True once the base has been still for still_time_s."""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.monotonic()
            fresh = self.last_odom is not None and now - self.last_odom < 0.5
            if fresh and self.still_since and now - self.still_since >= still_time_s:
                self.node.get_logger().info(
                    f'   base still after {now - start:.2f}s')
                return True
        why = 'still moving' if self.last_odom else 'no odometry'
        self.node.get_logger().warn(
            f'   base did not settle within {timeout_s:.0f}s ({why})')
        return False


class TaskFinished:
    """Decides when the running policy has finished its task.

    The arm goes back to its home pose in two situations: briefly, as a reset
    before retrying (object still there), and for good once the task is done
    (nothing left to do). So "finished" is a home visit that lasts longer than
    any reset pause - finished_dwell_s in mission.yaml.

    Positions only: at rest the arm jitters ~0.03 rad and its reported joint
    velocities spike to ~0.5 rad/s, so a velocity test would never see it as
    still. Being inside home_tolerance for the whole dwell is the stillness.
    """

    def __init__(self, node):
        self.node = node
        self.phase = None
        self.phase_time = None
        self.positions = {}
        node.create_subscription(TaskStatus, '/task/status', self._on_status, 10)
        node.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_status(self, msg):
        self.phase = msg.phase
        self.phase_time = time.monotonic()

    def _on_joints(self, msg):
        self.positions = dict(zip(msg.name, msg.position))

    def inferencing(self):
        return (self.phase == TaskStatus.INFERENCING
                and self.phase_time is not None
                and time.monotonic() - self.phase_time < STATUS_STALE_S)

    def wait(self, settings, label):
        """True when the task finished; False to abort. No time limit."""
        log = self.node.get_logger()
        home = settings['home_pose']
        enter_tol = float(settings['home_tolerance'])
        exit_tol = float(settings['home_exit_tolerance'])
        dwell_needed = float(settings['finished_dwell_s'])

        # 1. Inference has not really begun until the server reports it: the
        #    model loads first, with the arm parked at home the whole time.
        while not self.inferencing():
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # 2. Time home visits until one lasts long enough.
        #    Hysteresis: the arm arrives home inside enter_tol but only leaves
        #    beyond exit_tol, so a joint resting near enter_tol cannot flap.
        #    The arm starts at home, and that first stay is the policy getting
        #    going, not a finish - only a return after leaving counts.
        start = time.monotonic()
        at_home = False
        home_since = 0.0
        started = False     # the first departure from home is the task beginning
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if not self.inferencing():
                log.error(f'   {label}: inference stopped from outside')
                return False
            if any(j not in self.positions for j in ARM_JOINTS):
                continue

            errors = [abs(self.positions[j] - h) for j, h in zip(ARM_JOINTS, home)]
            worst = max(errors)
            now = time.monotonic()
            if at_home and worst > exit_tol:
                kind = 'reset' if started else 'task started'
                log.info(f'   left home after {now - home_since:.1f}s ({kind})')
                at_home = False
                started = True
            elif not at_home and worst <= enter_tol:
                at_home = True
                home_since = now
                if started:
                    log.info('   back home')
            elif at_home and started and now - home_since >= dwell_needed:
                log.info(f'   {label} finished - home for {now - home_since:.1f}s '
                         f'after {now - start:.1f}s')
                return True


class Actuator:
    """Arbiter: navigation and inference are never both active."""

    # physical_ai_server refuses every command when idle, with this message.
    # For a preventive stop that is the expected answer, not a failure.
    IDLE_MESSAGE = 'Not currently recording'

    def __init__(self, node, client, fps, still, finished, settings):
        self.node = node
        self.client = client
        self.fps = int(fps)
        self.still = still
        self.finished = finished
        self.settings = settings

    def _call(self, req, what, idle_ok=False):
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None:
            self.node.get_logger().error(f'{what}: /task/command timed out')
            return False
        if not res.success:
            if idle_ok and self.IDLE_MESSAGE in res.message:
                return True
            self.node.get_logger().error(f'{what}: refused -- {res.message}')
            return False
        return True

    def stop_policy(self):
        """FINISH, not STOP: only FINISH clears the server's on_inference flag.

        FINISH does not end the server's inference timer - that timer ends
        itself on its next tick. A START arriving before that tick leaves the
        old timer running beside the new one, forever: the policy then steps
        several times per frame and the arm jitters. So wait for the tick.
        """
        req = SendCommand.Request()
        req.command = SendCommand.Request.FINISH
        if not self._call(req, 'policy finish', idle_ok=True):
            return False
        # Ends on the tick's READY, or when the server goes quiet (stale).
        while self.finished.inferencing():
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return True

    def release(self):
        """Stop BOTH subsystems. Safe no-op when either is already idle."""
        self.node.cancelTask()
        return self.stop_policy()

    def navigate(self, pose_cfg, label):
        self.release()
        self.node.get_logger().info(
            f'nav -> {label} (x={pose_cfg["x"]:.2f}, y={pose_cfg["y"]:.2f}, '
            f'yaw={pose_cfg["yaw"]:.0f})')
        self.node.goToPose(make_pose(self.node, pose_cfg))

        last = 0.0
        while not self.node.isTaskComplete():
            fb = self.node.getFeedback()
            if fb and time.time() - last > 1.0:
                self.node.get_logger().info(
                    f'   {fb.distance_remaining:.2f} m remaining')
                last = time.time()

        if self.node.getResult() != TaskResult.SUCCEEDED:
            self.node.get_logger().error(f'nav to {label} FAILED')
            return False
        self.node.get_logger().info(f'   arrived at {label}')
        return True

    def settle(self, cfg):
        """Hold until the base has stopped coasting; never a fixed sleep."""
        return self.still.wait(float(cfg['still_time_s']),
                               float(cfg['settle_timeout_s']))

    def run_policy(self, path, instruction, label):
        """Run one ACT policy until it finishes its task, then stop it.

        Finished means the arm left home, came back and stayed (TaskFinished).
        There is no time limit.
        """
        # Never START while the previous policy's loop may still be alive.
        if not self.release():
            return False

        req = SendCommand.Request()
        req.command = SendCommand.Request.START_INFERENCE
        req.task_info.policy_path = path
        req.task_info.task_instruction = [instruction]
        req.task_info.fps = self.fps
        req.task_info.record_inference_mode = False

        self.node.get_logger().info(f'policy -> {label} ("{instruction}")')
        if not self._call(req, f'{label} start'):
            return False

        self.node.get_logger().info('   running until finished...')
        ok = self.finished.wait(self.settings, label)
        self.release()
        self.node.get_logger().info(f'   {label} done')
        return ok


def run_mission(act, cfg):
    poses = cfg['poses']
    man = cfg['policies']['manipulate']
    settings = cfg['settings']

    state = State.NAV_TO_PICK
    while state not in (State.DONE, State.ABORT):

        if state is State.NAV_TO_PICK:
            ok = act.navigate(poses['pick_area'], 'pick_area')
            ok = act.settle(settings) and ok
            state = State.PICK if ok else State.ABORT

        elif state is State.PICK:
            ok = act.run_policy(man['path'], man['instruction_pick'], 'pick')
            state = State.NAV_TO_PLACE if ok else State.ABORT

        elif state is State.NAV_TO_PLACE:
            ok = act.navigate(poses['place_area'], 'place_area')
            ok = act.settle(settings) and ok
            state = State.PLACE if ok else State.ABORT

        elif state is State.PLACE:
            ok = act.run_policy(man['path'], man['instruction_place'], 'place')
            state = State.NAV_TO_HOME if ok else State.ABORT

        elif state is State.NAV_TO_HOME:
            ok = act.navigate(poses['home'], 'home')
            state = State.DONE if ok else State.ABORT

    return state


def main():
    rclpy.init()
    nav = BasicNavigator()

    default_cfg = f"{get_package_share_directory('omniman_navigation')}/config/mission.yaml"
    nav.declare_parameter('mission_file', default_cfg)
    mission_file = nav.get_parameter('mission_file').value

    with open(mission_file) as f:
        cfg = yaml.safe_load(f)
    nav.get_logger().info(f'mission: {mission_file}')

    client = nav.create_client(SendCommand, '/task/command')
    if not client.wait_for_service(timeout_sec=10.0):
        nav.get_logger().error('/task/command unavailable -- physical_ai_server running?')
        nav.destroy_node()
        rclpy.shutdown()
        return

    # BasicNavigator.initial_pose defaults to a zero-norm quaternion, and
    # waitUntilNav2Active() publishes THAT to /initialpose until it hears back on
    # /amcl_pose - so skipping this clobbers a good AMCL estimate with garbage.
    # Assumes the robot is physically at home when launched.
    nav.setInitialPose(make_pose(nav, cfg['poses']['home']))
    nav.get_logger().info('waiting for Nav2...')
    nav.waitUntilNav2Active()

    act = Actuator(nav, client, cfg['settings']['fps'], BaseStill(nav),
                   TaskFinished(nav), cfg['settings'])
    try:
        final = run_mission(act, cfg)
        if final is State.DONE:
            nav.get_logger().info('mission complete')
        else:
            nav.get_logger().error('mission ABORTED')
    except KeyboardInterrupt:
        nav.get_logger().warn('interrupted')
    finally:
        # Never leave the robot driving or inferring on any exit path.
        act.release()
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
