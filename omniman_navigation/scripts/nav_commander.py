#!/usr/bin/env python3
"""
Send a single Nav2 goal from code, using the Simple Commander API.

Prereqs (already running):
  - nav2_launch.py
  - robot localized (AMCL)

Run:
  ros2 run omniman_navigation nav_commander.py
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

GOAL = {'x': 1.5, 'y': 0.5, 'yaw': 90.0}


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


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    navigator.goToPose(make_pose(navigator, GOAL['x'], GOAL['y'], GOAL['yaw']))

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'{feedback.distance_remaining:.2f} m remaining')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal reached.')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Goal canceled.')
    else:
        navigator.get_logger().error('Goal failed.')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
