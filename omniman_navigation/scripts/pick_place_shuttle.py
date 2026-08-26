#!/usr/bin/env python3
"""
Fixed home <-> target shuttle: drive out, pick, drive back, place.
    HOME (0, 0, 0)  --drive-->  TARGET (-1, 1, 0)  --policy: pick-->
    HOME (0, 0, 0)  <--policy: place--  <--drive--  TARGET

Runs as an explicit state machine:

Navigation and inference are strictly mutually exclusive. Nav2 drives the base
over /cmd_vel while the policy drives the arm over /leader/joint_trajectory --
different actuators, so nothing at the topic level stops both running at once.
Actuator is the arbiter: acquiring one releases the other first, and every exit
path (failure, Ctrl-C, crash) releases both.

No language resolution, no locations.yaml -- the two poses are hardcoded below
because for this run they're fixed and known. This is the simple counterpart to
vln_commander.py: same /task/command handoff, no instruction-matching layer.

Both legs trigger the SAME policy (the combined drive/pick/place skill) -- it
infers pick vs. place from gripper state and visual context, not from the task
instruction text, so no second checkpoint is needed. See docs/vla-training.md.

The policy is given a fixed run time rather than an explicit "done" signal --
physical_ai_server has no such signal to poll, so pick_duration_s / place_duration_s
must be long enough to cover the slowest real attempt. Tune them from how long the
episodes actually take when you watch it run.

Prereqs (already running):
  - nav2_launch.py (AMCL + map_server + planner/controller), robot localized
  - physical_ai_server_bringup.launch.py (serves /task/command)

Run:
  ros2 run omniman_navigation pick_place_shuttle.py
  ros2 run omniman_navigation pick_place_shuttle.py --ros-args \
      -p policy_path:=/home/dokterkepin/output/.../pretrained_model \
      -p pick_duration_s:=25.0 -p place_duration_s:=25.0
"""

import math
import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from physical_ai_interfaces.srv import SendCommand

HOME = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
TARGET = {'x': 1.0, 'y': -1.0, 'yaw': -100.0}


class State(Enum):
    """Mission progress. Each state hands off to the next, or to ABORT."""

    NAV_TO_TARGET = auto()
    PICK = auto()
    NAV_TO_HOME = auto()
    PLACE = auto()
    DONE = auto()
    ABORT = auto()


class Owner(Enum):
    """Which subsystem is currently allowed to actuate.

    Nav2 drives the base over /cmd_vel; the policy drives the arm over
    /leader/joint_trajectory. Different actuators, so nothing at the topic level
    stops both running at once -- the base could drive with the arm live, or the
    arm manipulate while the base creeps. This is what makes them exclusive.
    """

    NONE = auto()
    NAV = auto()
    POLICY = auto()


def make_pose(navigator, x, y, yaw_deg=0.0, frame_id='map'):
    """Same convention as nav_commander.py: planar pose, yaw in degrees."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class Actuator:
    """Arbiter that guarantees navigation and inference are never both active.

    Every acquire releases the other subsystem first, so the invariant holds even
    when a leg fails partway through or the script is interrupted.
    """

    def __init__(self, navigator, client, policy_path, fps):
        self.navigator = navigator
        self.client = client
        self.policy_path = policy_path
        self.fps = fps
        self.owner = Owner.NONE

    # physical_ai_server rejects every command when nothing is running, with this
    # message. For a preventive stop that is the expected answer, not a failure.
    IDLE_MESSAGE = 'Not currently recording'

    def _call(self, request, what, idle_ok=False):
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.navigator, future, timeout_sec=15.0)
        result = future.result()
        if result is None:
            self.navigator.get_logger().error(f'{what}: /task/command timed out')
            return False
        if not result.success:
            if idle_ok and self.IDLE_MESSAGE in result.message:
                # Nothing was running -- the goal (policy stopped) already holds.
                self.navigator.get_logger().info(f'{what}: nothing to stop')
                return True
            self.navigator.get_logger().error(f'{what}: refused -- {result.message}')
            return False
        self.navigator.get_logger().info(f'{what}: {result.message}')
        return True

    def stop_policy(self):
        """Stop inference on the server.

        Must be FINISH, not STOP: physical_ai_server's STOP only calls
        record_stop() and replies 'Recording stopped' -- it never clears
        on_inference, so the policy keeps publishing to the arm. FINISH is the
        only command that sets on_inference = False.

        Sent unconditionally rather than only when this script started the policy,
        because inference may already be running from the web UI when we launch.
        idle_ok: an idle server refuses it, which for a preventive stop is success.
        """
        req = SendCommand.Request()
        req.command = SendCommand.Request.FINISH
        return self._call(req, 'policy finish', idle_ok=True)

    def release(self):
        """Stop BOTH subsystems, regardless of what this script thinks it owns.

        Deliberately unconditional rather than keyed off self.owner: inference may
        already be running from the web UI when this script starts, in which case
        owner is NONE and an ownership-keyed release would drive off with the arm
        still live. Both calls are safe no-ops when idle -- cancelTask() guards on
        result_future, and FINISH just returns 'Not currently recording'.
        """
        self.navigator.cancelTask()
        self.stop_policy()
        self.owner = Owner.NONE

    def navigate(self, pose_cfg, label):
        """Drive to a pose. Blocks until Nav2 finishes. Returns True on success."""
        self.release()          # inference off before the base moves
        self.owner = Owner.NAV

        self.navigator.get_logger().info(
            f'nav -> {label}  (x={pose_cfg["x"]:.2f}, y={pose_cfg["y"]:.2f}, '
            f'yaw={pose_cfg["yaw"]:.0f})')
        self.navigator.goToPose(
            make_pose(self.navigator, pose_cfg['x'], pose_cfg['y'], pose_cfg['yaw']))

        last_log = 0.0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and time.time() - last_log > 1.0:
                self.navigator.get_logger().info(
                    f'   {feedback.distance_remaining:.2f} m remaining')
                last_log = time.time()

        result = self.navigator.getResult()
        self.owner = Owner.NONE  # Nav2 stopped driving on its own

        if result != TaskResult.SUCCEEDED:
            self.navigator.get_logger().error(f'nav to {label} failed: {result}')
            return False

        self.navigator.get_logger().info(f'nav: arrived at {label}')
        return True

    def run_policy(self, instruction, duration_s, label):
        """Run inference for a fixed time, then stop.

        physical_ai_server has no "policy finished" signal to wait on --
        START_INFERENCE runs until STOP is sent. duration_s is how long to trust
        it, not a real completion check.
        """
        self.release()          # base stopped before the arm moves

        req = SendCommand.Request()
        req.command = SendCommand.Request.START_INFERENCE
        req.task_info.policy_path = self.policy_path
        req.task_info.task_instruction = [instruction]
        req.task_info.fps = int(self.fps)
        req.task_info.record_inference_mode = False

        self.navigator.get_logger().info(f'policy -> {label} ({instruction})')
        if not self._call(req, f'{label} start'):
            return False

        self.owner = Owner.POLICY
        self.navigator.get_logger().info(
            f'policy: running {label} for {duration_s:.0f}s...')
        time.sleep(duration_s)

        self.release()
        return True


def run_mission(actuator, pick_duration_s, place_duration_s):
    """Drive the state machine. Each state returns the next one."""
    state = State.NAV_TO_TARGET

    while state not in (State.DONE, State.ABORT):
        if state is State.NAV_TO_TARGET:
            state = State.PICK if actuator.navigate(TARGET, 'target') else State.ABORT

        elif state is State.PICK:
            ok = actuator.run_policy('pick the object', pick_duration_s, 'pick')
            state = State.NAV_TO_HOME if ok else State.ABORT

        elif state is State.NAV_TO_HOME:
            state = State.PLACE if actuator.navigate(HOME, 'home') else State.ABORT

        elif state is State.PLACE:
            ok = actuator.run_policy('place the object', place_duration_s, 'place')
            state = State.DONE if ok else State.ABORT

    return state


def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.declare_parameter(
        'policy_path',
        '/home/dokterkepin/output/omniman_drive_pick_place_v3/checkpoints/100000/pretrained_model')
    navigator.declare_parameter('fps', 30)
    navigator.declare_parameter('pick_duration_s', 20.0)
    navigator.declare_parameter('place_duration_s', 20.0)

    policy_path = navigator.get_parameter('policy_path').value
    fps = navigator.get_parameter('fps').value
    pick_duration_s = navigator.get_parameter('pick_duration_s').value
    place_duration_s = navigator.get_parameter('place_duration_s').value

    # A -p policy_path:=... on the command line silently overrides the default
    # above with no warning if it doesn't exist -- this makes it obvious which
    # one is actually in effect before the run gets far enough to fail on it.
    navigator.get_logger().info(f'Using policy_path: {policy_path}')

    client = navigator.create_client(SendCommand, '/task/command')
    if not client.wait_for_service(timeout_sec=10.0):
        navigator.get_logger().error(
            '/task/command unavailable -- is physical_ai_server running?')
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # BasicNavigator.initial_pose defaults to an empty PoseStamped (an invalid,
    # zero-norm quaternion). waitUntilNav2Active() publishes THAT to /initialpose
    # in a loop until it hears back on /amcl_pose -- so skipping setInitialPose()
    # here means AMCL's belief about the robot gets clobbered with garbage the
    # moment this script starts, even if it was already well localized.
    #
    # This assumes the robot is physically at HOME when the script is launched --
    # true for this shuttle's own home<->target<->home cycle, but re-check it if
    # you ever start the script mid-cycle or from a different physical position.
    navigator.setInitialPose(make_pose(navigator, HOME['x'], HOME['y'], HOME['yaw']))

    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()

    actuator = Actuator(navigator, client, policy_path, fps)
    try:
        final = run_mission(actuator, pick_duration_s, place_duration_s)
        if final is State.DONE:
            navigator.get_logger().info('Pick-and-place cycle complete.')
        else:
            navigator.get_logger().error('Mission aborted.')
    except KeyboardInterrupt:
        navigator.get_logger().warn('Interrupted.')
    finally:
        # The robot must not be left driving or inferring on any exit path,
        # including Ctrl-C partway through a policy run.
        actuator.release()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
