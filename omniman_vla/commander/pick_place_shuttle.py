#!/usr/bin/env python3
"""
Pick-and-place shuttle with nav2 + two ACT policies.

    home -> pick_area   nav2
            correct     base_correction policy
            pick        manipulate policy
    ->      place_area  nav2
            correct     base_correction policy
            place       manipulate policy (same checkpoint)
    -> home             nav2

Everything tunable lives in config/mission.yaml - poses, policy paths,
instructions, durations. Nothing here needs editing to change the mission.

See pick_place_mission.py for the same mission without the correction step.

WAITING, NOT SLEEPING
    After each nav leg the mission waits for wheel odometry to show the base
    has actually stopped (BaseStill), rather than sleeping a fixed settle_s.

WHY A CORRECTION STEP
    Nav2 stops when SimpleGoalChecker is satisfied, which on this robot means up
    to ~5.7 deg and ~12 cm of residual error - and it cannot do better, because
    AMCL only knows the yaw to ~4.7 deg (1 sigma). The base_correction policy
    closes that gap visually, so it is not bound by the map-frame estimate.

MUTUAL EXCLUSION
    Nav2 drives the base over /cmd_vel; the policies drive the base over the SAME
    topic (leader_mobile publishes Twist) plus the arm over /leader/joint_trajectory.
    They must never run together. Actuator guarantees that: every acquire releases
    the other first, and every exit path releases both.

Prereqs:
  - nav2_launch.py, robot localized
  - physical_ai_server_bringup.launch.py (serves /task/command)

Run:
  ros2 run omniman_vla pick_place_shuttle.py
  ros2 run omniman_vla pick_place_shuttle.py --ros-args \
      -p mission_file:=/path/to/mission.yaml
"""

import math
import os
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

# "Still" means every odometry twist component under these for still_time_s.
# Same test as pick_place_mission.py uses.
STILL_LINEAR = 0.01     # m/s
STILL_ANGULAR = 0.02    # rad/s

# physical_ai_server publishes /task/status on every inference tick (~30 Hz)
# and goes quiet once inference ends, so an old INFERENCING is not trusted.
STATUS_STALE_S = 1.0
# How long a FINISH may take to actually end the server's inference loop.
STOP_TIMEOUT_S = 3.0


def load_poses(mission_file):
    """Places from poses.yaml beside the mission file (saved from the web UI)."""
    path = os.path.join(os.path.dirname(os.path.realpath(mission_file)), 'poses.yaml')
    with open(path) as f:
        return yaml.safe_load(f) or {}


class State(Enum):
    NAV_TO_PICK = auto()
    CORRECT_AT_PICK = auto()
    PICK = auto()
    NAV_TO_PLACE = auto()
    CORRECT_AT_PLACE = auto()
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


class Actuator:
    """Arbiter: navigation and inference are never both active."""

    # physical_ai_server refuses every command when idle, with this message.
    # For a preventive stop that is the expected answer, not a failure.
    IDLE_MESSAGE = 'Not currently recording'

    def __init__(self, node, client, fps, still):
        self.node = node
        self.client = client
        self.fps = int(fps)
        self.still = still
        self.phase = None
        self.phase_time = None
        node.create_subscription(TaskStatus, '/task/status', self._on_status, 10)

    def _on_status(self, msg):
        self.phase = msg.phase
        self.phase_time = time.monotonic()

    def inferencing(self):
        return (self.phase == TaskStatus.INFERENCING
                and self.phase_time is not None
                and time.monotonic() - self.phase_time < STATUS_STALE_S)

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
        start = time.monotonic()
        while self.inferencing():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if time.monotonic() - start > STOP_TIMEOUT_S:
                self.node.get_logger().error(
                    'policy finish: server still inferencing')
                return False
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

    def run_policy(self, path, instruction, duration_s, label):
        """Run one ACT policy for a fixed time, then stop it.

        physical_ai_server exposes no "policy finished" signal, so duration_s is
        how long to trust it, not a completion check.
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

        self.node.get_logger().info(f'   running {duration_s:.0f}s...')
        time.sleep(float(duration_s))
        self.release()
        self.node.get_logger().info(f'   {label} done')
        return True


def run_mission(act, cfg):
    poses = cfg['poses']
    corr = cfg['policies']['base_correction']
    man = cfg['policies']['manipulate']
    settings = cfg['settings']

    def correct(where):
        return act.run_policy(corr['path'], corr['instruction'],
                              corr['duration_s'], f'correct @ {where}')

    state = State.NAV_TO_PICK
    while state not in (State.DONE, State.ABORT):

        if state is State.NAV_TO_PICK:
            ok = act.navigate(poses['pick_area'], 'pick_area')
            ok = act.settle(settings) and ok
            state = State.CORRECT_AT_PICK if ok else State.ABORT

        elif state is State.CORRECT_AT_PICK:
            state = State.PICK if correct('pick_area') else State.ABORT

        elif state is State.PICK:
            ok = act.run_policy(man['path'], man['instruction_pick'],
                                man['pick_duration_s'], 'pick')
            state = State.NAV_TO_PLACE if ok else State.ABORT

        elif state is State.NAV_TO_PLACE:
            ok = act.navigate(poses['place_area'], 'place_area')
            ok = act.settle(settings) and ok
            state = State.CORRECT_AT_PLACE if ok else State.ABORT

        elif state is State.CORRECT_AT_PLACE:
            state = State.PLACE if correct('place_area') else State.ABORT

        elif state is State.PLACE:
            ok = act.run_policy(man['path'], man['instruction_place'],
                                man['place_duration_s'], 'place')
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
    cfg['poses'] = load_poses(mission_file)
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

    act = Actuator(nav, client, cfg['settings']['fps'], BaseStill(nav))
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
