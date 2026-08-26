#!/usr/bin/env python3
"""
Fixed home <-> target shuttle: drive out, pick, drive back, place.

    HOME (0, 0, 0)  --drive-->  TARGET (-1, 1, 0)  --policy: pick-->
                                                          |
    HOME (0, 0, 0)  <--policy: place--  <--drive--  TARGET

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

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from physical_ai_interfaces.srv import SendCommand

HOME = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
TARGET = {'x': 1.0, 'y': -1.0, 'yaw': -100.0}


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


def drive_to(navigator, pose_cfg, label):
    navigator.get_logger().info(
        f'-> driving to {label}  (x={pose_cfg["x"]:.2f}, y={pose_cfg["y"]:.2f}, '
        f'yaw={pose_cfg["yaw"]:.0f})')

    navigator.goToPose(make_pose(navigator, pose_cfg['x'], pose_cfg['y'], pose_cfg['yaw']))

    last_log = 0.0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and time.time() - last_log > 1.0:
            navigator.get_logger().info(f'   {feedback.distance_remaining:.2f} m remaining')
            last_log = time.time()

    result = navigator.getResult()
    if result != TaskResult.SUCCEEDED:
        navigator.get_logger().error(f'navigation to {label} did not succeed: {result}')
        return False

    navigator.get_logger().info(f'arrived at {label}')
    return True


def call_task_command(navigator, client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=15.0)
    result = future.result()
    if result is None:
        navigator.get_logger().error('/task/command call timed out')
        return False
    if not result.success:
        navigator.get_logger().error(f'/task/command refused: {result.message}')
        return False
    navigator.get_logger().info(result.message)
    return True


def run_policy(navigator, client, policy_path, instruction, fps, duration_s, label):
    """Start inference, let it run for a fixed time, then stop.

    physical_ai_server has no "policy finished" signal to wait on -- START_INFERENCE
    just runs until STOP is sent. duration_s is how long to trust it, not a real
    completion check.
    """
    req = SendCommand.Request()
    req.command = SendCommand.Request.START_INFERENCE
    req.task_info.policy_path = policy_path
    req.task_info.task_instruction = [instruction]
    req.task_info.fps = int(fps)
    req.task_info.record_inference_mode = False

    navigator.get_logger().info(f'{label}: starting policy ({instruction})')
    if not call_task_command(navigator, client, req):
        return False

    navigator.get_logger().info(f'{label}: running for {duration_s:.0f}s...')
    time.sleep(duration_s)

    stop_req = SendCommand.Request()
    stop_req.command = SendCommand.Request.STOP
    navigator.get_logger().info(f'{label}: stopping policy')
    return call_task_command(navigator, client, stop_req)


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

    try:
        if not drive_to(navigator, TARGET, 'target'):
            return
        if not run_policy(navigator, client, policy_path, 'pick the object',
                          fps, pick_duration_s, 'pick'):
            return
        if not drive_to(navigator, HOME, 'home'):
            return
        run_policy(navigator, client, policy_path, 'place the object',
                   fps, place_duration_s, 'place')
        navigator.get_logger().info('Pick-and-place cycle complete.')
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
