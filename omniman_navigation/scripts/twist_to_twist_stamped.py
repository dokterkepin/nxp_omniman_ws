#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__("twist_to_twist_stamped")
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, "cmd_vel_stamped", 10)

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(TwistToTwistStamped())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
