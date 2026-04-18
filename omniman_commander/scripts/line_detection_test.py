#!/usr/bin/env python3
# Detection-only sibling of line_follower_pid.py.
#
# Runs the exact same vision pipeline (ROI, black-line mask, centroid, red
# mask, normalized error) and shows the annotated window — but never
# publishes to /cmd_vel. Use this to verify detection before letting the
# robot drive.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LineDetectionTest(Node):
    def __init__(self):
        super().__init__("line_detection_test")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("debug_image_topic", "/line_follower/debug_image")

        self.declare_parameter("roi_top_frac", 0.55)
        self.declare_parameter("roi_bot_frac", 1.0)
        self.declare_parameter("black_thresh", 70)
        self.declare_parameter("min_line_area_px", 300)

        self.declare_parameter("red_area_stop_frac", 0.22)
        self.declare_parameter("red_area_slow_frac", 0.05)

        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "line_detection_test")

        self.image_topic = self.get_parameter("image_topic").value
        self.debug_topic = self.get_parameter("debug_image_topic").value
        self.roi_top = float(self.get_parameter("roi_top_frac").value)
        self.roi_bot = float(self.get_parameter("roi_bot_frac").value)
        self.black_thresh = int(self.get_parameter("black_thresh").value)
        self.min_area = float(self.get_parameter("min_line_area_px").value)
        self.red_stop = float(self.get_parameter("red_area_stop_frac").value)
        self.red_slow = float(self.get_parameter("red_area_slow_frac").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.bridge = CvBridge()
        self.frames = 0

        if self.show_window:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 640, 480)
            except Exception as e:
                self.get_logger().warn(f"cv2 window init failed ({e}); disabling show_window.")
                self.show_window = False

        self.debug_pub = self.create_publisher(Image, self.debug_topic, 1)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"line_detection_test up: image={self.image_topic} "
            f"black_thresh={self.black_thresh} roi_top={self.roi_top} "
            f"(DETECTION ONLY — no /cmd_vel published)"
        )

    def on_image(self, msg: Image):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = cv.shape[:2]
        y1 = max(0, int(h * self.roi_top))
        y2 = min(h, int(h * self.roi_bot))
        if y2 <= y1:
            return
        roi = cv[y1:y2]

        hsv_full = cv2.cvtColor(cv, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv_full, (0, 110, 80), (10, 255, 255))
        red2 = cv2.inRange(hsv_full, (170, 110, 80), (180, 255, 255))
        red_mask = cv2.bitwise_or(red1, red2)
        red_frac = float(cv2.countNonZero(red_mask)) / float(h * w)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, black = cv2.threshold(gray, self.black_thresh, 255, cv2.THRESH_BINARY_INV)
        k = np.ones((3, 3), np.uint8)
        black = cv2.erode(black, k, iterations=1)
        black = cv2.dilate(black, k, iterations=2)

        cx = None
        cy = None
        err = 0.0
        line_found = False
        M = cv2.moments(black)
        if M["m00"] > self.min_area:
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            err = (cx - w / 2.0) / (w / 2.0)
            line_found = True

        dbg = cv.copy()
        overlay = cv2.cvtColor(black, cv2.COLOR_GRAY2BGR)
        dbg[y1:y2] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
        red_tint = np.zeros_like(dbg)
        red_tint[:, :, 2] = red_mask
        dbg = cv2.addWeighted(dbg, 1.0, red_tint, 0.4, 0)
        cv2.line(dbg, (0, y1), (w, y1), (0, 200, 200), 1)
        cv2.line(dbg, (0, y2 - 1), (w, y2 - 1), (0, 200, 200), 1)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        if cx is not None:
            cv2.circle(dbg, (int(cx), int(cy + y1)), 8, (0, 255, 0), -1)
            cv2.line(dbg, (w // 2, int(cy + y1)), (int(cx), int(cy + y1)),
                     (0, 255, 0), 2)
        status = "GOAL" if red_frac >= self.red_stop else (
            "SLOW" if red_frac >= self.red_slow else "RUN")
        cv2.putText(
            dbg,
            f"err={err:+.2f} red={red_frac:.2f} line={line_found} {status} (DETECT)",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        if self.debug_pub.get_subscription_count() > 0:
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
            except Exception:
                pass

        if self.show_window:
            try:
                cv2.imshow(self.window_name, dbg)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"cv2.imshow failed ({e}); disabling show_window.")
                self.show_window = False

        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info(
                f"frame {self.frames}: err={err:+.2f} red={red_frac:.2f} "
                f"line_found={line_found} status={status}"
            )


def main():
    rclpy.init()
    node = LineDetectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
