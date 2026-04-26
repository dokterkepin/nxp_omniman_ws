#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self._bridge = CvBridge()
        self.create_subscription(Image, "/image_raw", self._on_image, 10)
        self.get_logger().info("Camera viewer ready — waiting for /image_raw")

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
