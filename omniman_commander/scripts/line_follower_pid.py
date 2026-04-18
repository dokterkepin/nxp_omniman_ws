#!/usr/bin/env python3
# Vision PID line-follower for the mecanum base.
#
# Subscribes to the palm-mounted USB camera image, finds the black tape line
# in a near-field ROI, and drives /cmd_vel (TwistStamped) so the line stays
# centered horizontally. A red patch (the goal square) filling a configurable
# fraction of the frame slows then stops the robot.
#
# Lateral correction is applied as body-frame strafe by default (mecanum), or
# as yaw if use_strafe:=false. Tune kp/kd and error_sign for your mount.

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LineFollowerPID(Node):
    def __init__(self):
        super().__init__("line_follower_pid")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_image_topic", "/line_follower/debug_image")

        self.declare_parameter("forward_speed", 0.06)
        self.declare_parameter("approach_speed", 0.02)
        self.declare_parameter("max_lateral_speed", 0.08)
        self.declare_parameter("use_strafe", True)
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("error_sign", -1.0)
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("roi_top_frac", 0.55)
        self.declare_parameter("roi_bot_frac", 1.0)
        self.declare_parameter("black_thresh", 70)
        self.declare_parameter("min_line_area_px", 300)

        self.declare_parameter("red_area_stop_frac", 0.22)
        self.declare_parameter("red_area_slow_frac", 0.05)
        self.declare_parameter("stop_and_exit", True)

        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "line_follower_pid")

        self.image_topic = self.get_parameter("image_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.debug_topic = self.get_parameter("debug_image_topic").value
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.approach_speed = float(self.get_parameter("approach_speed").value)
        self.max_lat = float(self.get_parameter("max_lateral_speed").value)
        self.use_strafe = bool(self.get_parameter("use_strafe").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.err_sign = float(self.get_parameter("error_sign").value)
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.roi_top = float(self.get_parameter("roi_top_frac").value)
        self.roi_bot = float(self.get_parameter("roi_bot_frac").value)
        self.black_thresh = int(self.get_parameter("black_thresh").value)
        self.min_area = float(self.get_parameter("min_line_area_px").value)
        self.red_stop = float(self.get_parameter("red_area_stop_frac").value)
        self.red_slow = float(self.get_parameter("red_area_slow_frac").value)
        self.stop_and_exit = bool(self.get_parameter("stop_and_exit").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)

        if self.show_window:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 640, 480)
            except Exception as e:
                self.get_logger().warn(f"cv2 window init failed ({e}); disabling show_window.")
                self.show_window = False

        self.bridge = CvBridge()
        self.latest_err = 0.0
        self.prev_err = 0.0
        self.integral = 0.0
        self.last_t = None
        self.goal_reached = False
        self.red_frac = 0.0
        self.line_found = False
        self.img_received = False

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 1)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_tick)

        self.get_logger().info(
            f"line_follower_pid up: image={self.image_topic} cmd={self.cmd_topic} "
            f"use_strafe={self.use_strafe} fwd={self.forward_speed:.2f} "
            f"kp={self.kp} ki={self.ki} kd={self.kd} err_sign={self.err_sign}"
        )

    def on_image(self, msg: Image):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return
        self.img_received = True

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
        self.red_frac = float(cv2.countNonZero(red_mask)) / float(h * w)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, black = cv2.threshold(gray, self.black_thresh, 255, cv2.THRESH_BINARY_INV)
        k = np.ones((3, 3), np.uint8)
        black = cv2.erode(black, k, iterations=1)
        black = cv2.dilate(black, k, iterations=2)

        cx = None
        cy = None
        M = cv2.moments(black)
        if M["m00"] > self.min_area:
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            self.latest_err = (cx - w / 2.0) / (w / 2.0)
            self.line_found = True
        else:
            self.line_found = False

        want_debug = self.show_window or self.debug_pub.get_subscription_count() > 0
        if want_debug:
            dbg = cv.copy()
            overlay = cv2.cvtColor(black, cv2.COLOR_GRAY2BGR)
            dbg[y1:y2] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
            # red mask overlay (red tint where red pixels are)
            red_tint = np.zeros_like(dbg)
            red_tint[:, :, 2] = red_mask
            dbg = cv2.addWeighted(dbg, 1.0, red_tint, 0.4, 0)
            # ROI edges
            cv2.line(dbg, (0, y1), (w, y1), (0, 200, 200), 1)
            cv2.line(dbg, (0, y2 - 1), (w, y2 - 1), (0, 200, 200), 1)
            # image center line
            cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
            # centroid
            if cx is not None:
                cv2.circle(dbg, (int(cx), int(cy + y1)), 8, (0, 255, 0), -1)
                cv2.line(dbg, (w // 2, int(cy + y1)), (int(cx), int(cy + y1)),
                         (0, 255, 0), 2)
            # status text
            status = "GOAL" if self.red_frac >= self.red_stop else (
                "SLOW" if self.red_frac >= self.red_slow else "RUN")
            cv2.putText(
                dbg,
                f"err={self.latest_err:+.2f} red={self.red_frac:.2f} "
                f"line={self.line_found} {status}",
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

    def on_tick(self):
        if self.goal_reached:
            self.publish_stop()
            return

        if not self.img_received:
            return

        if self.red_frac >= self.red_stop:
            self.get_logger().info(
                f"goal reached: red_frac={self.red_frac:.2f} >= {self.red_stop:.2f}. stopping."
            )
            self.goal_reached = True
            self.publish_stop()
            if self.stop_and_exit:
                rclpy.shutdown()
            return

        if not self.line_found:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = 0.5 * self.forward_speed
            self.cmd_pub.publish(cmd)
            self.integral = 0.0
            self.prev_err = 0.0
            return

        now = time.monotonic()
        dt = 1.0 / self.rate_hz if self.last_t is None else max(1e-3, now - self.last_t)
        self.last_t = now

        err = self.err_sign * self.latest_err
        self.integral = max(-1.0, min(1.0, self.integral + err * dt))
        deriv = (err - self.prev_err) / dt
        self.prev_err = err

        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        out = max(-self.max_lat, min(self.max_lat, out))

        fwd = self.forward_speed
        if self.red_frac >= self.red_slow:
            t = (self.red_frac - self.red_slow) / max(1e-6, self.red_stop - self.red_slow)
            t = max(0.0, min(1.0, t))
            fwd = self.forward_speed + (self.approach_speed - self.forward_speed) * t

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = fwd
        if self.use_strafe:
            cmd.twist.linear.y = out
        else:
            cmd.twist.angular.z = out
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
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
