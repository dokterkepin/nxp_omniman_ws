#!/usr/bin/env python3
"""
Print the current robot position from AMCL.

Prereqs (already running):
  - nav2_launch.py (starts AMCL + map_server)

Run:
  ros2 run omniman_navigation get_robot_pose.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    """Extract yaw (rotation around Z) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._callback,
            10,
        )
        self.get_logger().info('Waiting for /amcl_pose ...')

    def _callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        x = p.position.x
        y = p.position.y
        yaw_deg = math.degrees(yaw_from_quaternion(p.orientation))
        self.get_logger().info(
            f'x={x:.3f} m  y={y:.3f} m  yaw={yaw_deg:.1f} deg'
        )


def main():
    rclpy.init()
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
