#!/usr/bin/env python3
"""
Shuttle between HOME and GOAL, repeatedly.

Drives HOME -> GOAL -> HOME and counts that as one lap, for LAPS laps.

Prereqs (already running):
  - nav2_launch.py
  - robot localized (AMCL)

Run:
  ros2 run omniman_navigation nav_commander.py
  ros2 run omniman_navigation nav_commander.py --ros-args -p laps:=10
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

HOME = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
GOAL = {'x': 0.2, 'y': 1.4, 'yaw': 0.0}
LAPS = 100


def make_pose(navigator, x, y, yaw_deg=0.0, frame_id='map'):
    """Planar pose, yaw in degrees."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def go(navigator, target, label):
    """Drive to one pose. Blocks until done. True on success."""
    navigator.get_logger().info(
        f'-> {label} (x={target["x"]:.2f}, y={target["y"]:.2f}, '
        f'yaw={target["yaw"]:.0f})')
    navigator.goToPose(
        make_pose(navigator, target['x'], target['y'], target['yaw']))

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'   {feedback.distance_remaining:.2f} m remaining')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f'   arrived at {label}')
        return True

    navigator.get_logger().error(f'   {label} failed: {result}')
    return False


def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.declare_parameter('laps', LAPS)
    laps = navigator.get_parameter('laps').value

    navigator.waitUntilNav2Active()

    completed = 0
    try:
        for lap in range(1, laps + 1):
            navigator.get_logger().info(f'===== lap {lap}/{laps} =====')

            if not go(navigator, GOAL, 'goal'):
                break
            if not go(navigator, HOME, 'home'):
                break

            completed = lap
    except KeyboardInterrupt:
        navigator.get_logger().warn('Interrupted.')
    finally:
        # Do not leave the robot driving on any exit path.
        navigator.cancelTask()

    navigator.get_logger().info(f'Completed {completed}/{laps} laps.')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
