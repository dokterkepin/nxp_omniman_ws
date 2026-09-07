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
  ros2 run omniman_navigation pick_place_shuttle.py
  ros2 run omniman_navigation pick_place_shuttle.py --ros-args \
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
from physical_ai_interfaces.srv import SendCommand


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


class Actuator:
    """Arbiter: navigation and inference are never both active."""

    # physical_ai_server refuses every command when idle, with this message.
    # For a preventive stop that is the expected answer, not a failure.
    IDLE_MESSAGE = 'Not currently recording'

    def __init__(self, node, client, fps):
        self.node = node
        self.client = client
        self.fps = int(fps)

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
        """FINISH, not STOP: only FINISH clears the server's on_inference flag."""
        req = SendCommand.Request()
        req.command = SendCommand.Request.FINISH
        return self._call(req, 'policy finish', idle_ok=True)

    def release(self):
        """Stop BOTH subsystems. Safe no-op when either is already idle."""
        self.node.cancelTask()
        self.stop_policy()

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

    def run_policy(self, path, instruction, duration_s, label):
        """Run one ACT policy for a fixed time, then stop it.

        physical_ai_server exposes no "policy finished" signal, so duration_s is
        how long to trust it, not a completion check.
        """
        self.release()

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
    settle = float(cfg['settings']['settle_s'])

    def correct(where):
        return act.run_policy(corr['path'], corr['instruction'],
                              corr['duration_s'], f'correct @ {where}')

    state = State.NAV_TO_PICK
    while state not in (State.DONE, State.ABORT):

        if state is State.NAV_TO_PICK:
            ok = act.navigate(poses['pick_area'], 'pick_area')
            time.sleep(settle)
            state = State.CORRECT_AT_PICK if ok else State.ABORT

        elif state is State.CORRECT_AT_PICK:
            state = State.PICK if correct('pick_area') else State.ABORT

        elif state is State.PICK:
            ok = act.run_policy(man['path'], man['instruction_pick'],
                                man['pick_duration_s'], 'pick')
            state = State.NAV_TO_PLACE if ok else State.ABORT

        elif state is State.NAV_TO_PLACE:
            ok = act.navigate(poses['place_area'], 'place_area')
            time.sleep(settle)
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

    act = Actuator(nav, client, cfg['settings']['fps'])
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
