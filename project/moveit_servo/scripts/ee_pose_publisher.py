#!/usr/bin/env python3
"""Publish the composed base_link → ee_link transform as a Pose topic for PlotJuggler."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class EEPosePublisher(Node):
    def __init__(self):
        super().__init__('ee_pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/ee_pose', 10)
        self.timer = self.create_timer(0.02, self.publish_pose)  # 50 Hz

    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'ee_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        msg = PoseStamped()
        msg.header = t.header
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = EEPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
