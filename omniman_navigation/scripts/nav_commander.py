#!/usr/bin/env python3
"""
Minimal Nav2 mission example using the Simple Commander API (BasicNavigator).

This is the programmatic equivalent of clicking "2D Goal Pose" in RViz, but
from code. BasicNavigator wraps the Nav2 action servers (/navigate_to_pose,
/navigate_through_poses, /follow_waypoints, ...) so you don't have to write
raw rclcpp_action / rclpy.action clients yourself.

Prereqs (already running in your stack):
  - The full Nav2 bringup (planner, controller, bt_navigator, lifecycle mgr)
  - A localization source publishing map -> odom (your EKF + slam_toolbox,
    or AMCL if you switch to a saved map).

Run it (after `colcon build` + sourcing the workspace):
  ros2 run omniman_navigation nav_commander.py
"""

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw_deg=0.0, frame_id="map"):
    """Build a PoseStamped goal in the map frame from x, y and a yaw in degrees."""
    import math

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    # Quaternion from yaw only (planar robot): z = sin(yaw/2), w = cos(yaw/2)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # ---- 1. Wait until Nav2 is fully up (all lifecycle nodes active) --------
    # If you rely on EKF/slam_toolbox for the map->odom transform you do NOT
    # need to call setInitialPose(). Only AMCL needs an initial pose. Uncomment
    # the block below if you switch to AMCL localization on a saved map.
    #
    # initial = make_pose(navigator, 0.0, 0.0, 0.0)
    # navigator.setInitialPose(initial)

    navigator.waitUntilNav2Active()  # blocks until planner/controller/bt are active

    # ---- 2. Send a single goal ---------------------------------------------
    goal = make_pose(navigator, 1.5, 0.5, yaw_deg=90.0)
    navigator.goToPose(goal)

    # ---- 3. Spin while the task runs, reading live feedback -----------------
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            remaining = feedback.distance_remaining
            navigator.get_logger().info(f"Distance remaining: {remaining:.2f} m")

            # Example of a timeout / cancel policy: bail out after 90 s.
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=90.0):
                navigator.cancelTask()

    # ---- 4. Check how it ended ---------------------------------------------
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info("Goal reached.")
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn("Goal was canceled.")
    elif result == TaskResult.FAILED:
        navigator.get_logger().error("Goal failed (planner/controller could not reach it).")

    # ---- 5. (Optional) Patrol through several waypoints ---------------------
    # waypoints = [
    #     make_pose(navigator, 1.5, 0.5, 90.0),
    #     make_pose(navigator, 1.5, 2.0, 180.0),
    #     make_pose(navigator, 0.0, 2.0, -90.0),
    #     make_pose(navigator, 0.0, 0.0, 0.0),
    # ]
    # navigator.followWaypoints(waypoints)
    # while not navigator.isTaskComplete():
    #     fb = navigator.getFeedback()
    #     if fb:
    #         navigator.get_logger().info(f"On waypoint {fb.current_waypoint + 1}")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
