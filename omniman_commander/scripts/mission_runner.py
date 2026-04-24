#!/usr/bin/env python3
# Vision + odometry mission runner — state-machine version.
#
# The mission is a list of step dicts; one ROS timer ticks the current step
# until it reports done, then advances. No threads, no blocking loops.
#
# Step kinds:
#   STRAIGHT  — drive along base_link +x (or -x if distance<0) until odom
#               distance reached. Signed distance: + = forward, - = reverse.
#               Optional follow_line (forward only).
#   STRAFE    — pure lateral motion (+y=left, -y=right), no line PID, no
#               forward component. All four mecanum wheels engage.
#   TURN      — rotate base to (start_yaw + dtheta)
#   COLOR     — hold still until the HSV detector flips the color-found flag,
#               then advance. No search motion — assume the preceding TURN/
#               STRAFE already points the camera at the swatch. Do approach/
#               dwell/retreat with STRAIGHT steps.
#   DWELL     — publish zero twist for N seconds
#
# Edit build_mission() below to change the course.

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


# HSV ranges, OpenCV convention: H 0-180, S 0-255, V 0-255.
# Arena swatch reference values (measured across bright + shaded regions):
#   yellow #DEAD33 -> HSV(21, 196, 222)   (shaded #B18225 -> HSV(20, 202, 177))
#   blue   #474868 -> HSV(119,  81, 104)  (shaded #32364C -> HSV(115,  87,  76))
#   green  #73C66E -> HSV(58,  113, 198)  (shaded #417642 -> HSV(60,  114, 118))
#
# Night/low-light tuning: lower the V floor (darker) and S floor (desaturated
# under weak light) so the swatch still registers. Upper bounds stay at 255.
# If false positives creep in, raise S floor back up first (V floor is the
# one that actually captures dark-but-still-blue pixels). Blue is a
# low-saturation navy here, so keep its S floor low.
COLOR_HSV = {
    "yellow": [((10,  80, 100), (32, 255, 255))],
    "blue":   [((100, 30,  40), (135, 180, 180))],
    "green":  [((45,  50,  60), (75, 255, 255))],
}


STEP_STRAIGHT = "STRAIGHT"
STEP_STRAFE = "STRAFE"
STEP_TURN = "TURN"
STEP_COLOR = "COLOR"
STEP_DWELL = "DWELL"


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def mask_for_color(hsv_img, color_name):
    ranges = COLOR_HSV.get(color_name)
    if not ranges:
        raise ValueError(f"unknown color: {color_name}")
    m = None
    for (lo, hi) in ranges:
        sub = cv2.inRange(hsv_img, lo, hi)
        m = sub if m is None else cv2.bitwise_or(m, sub)
    return m


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MissionRunner(Node):
    def __init__(self):
        super().__init__("mission_runner")

        # topics
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/mecanum_drive_controller/odometry")
        self.declare_parameter("debug_image_topic", "/mission_runner/debug_image")
        self.declare_parameter("state_topic", "/mission_runner/state")

        # motion
        self.declare_parameter("forward_speed", 0.06)
        self.declare_parameter("approach_speed", 0.03)
        self.declare_parameter("coast_speed", 0.03)
        self.declare_parameter("max_lateral_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.6)
        self.declare_parameter("control_rate_hz", 20.0)

        # PID
        self.declare_parameter("line_kp", 0.4)
        self.declare_parameter("line_kd", 0.05)
        self.declare_parameter("error_sign", -1.0)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("yaw_tolerance_rad", 0.02)
        # multiplier on every TURN dtheta. Set to -1.0 if odom and cmd_vel yaw
        # conventions disagree (i.e. "LEFT" rotates the robot right in reality).
        self.declare_parameter("turn_sign", 1.0)

        # vision
        self.declare_parameter("roi_top_frac", 0.55)
        self.declare_parameter("roi_bot_frac", 1.0)
        self.declare_parameter("black_thresh", 70)
        self.declare_parameter("min_line_area_px", 300)
        # Lane-keeping: when only one lane edge is visible, apply this
        # synthetic lateral error so the PID steers AWAY from the visible
        # line (toward the road center). Normalized to half-frame. Larger
        # = stronger corrective lateral push. 0.5 ≈ aim for a quarter-frame
        # offset on the opposite side.
        self.declare_parameter("single_line_offset", 0.5)
        # Dual-line acceptance: require L and R centroids to be at least
        # this fraction of frame width apart before trusting "two lines".
        # Otherwise it's probably one line straddling the split -> treat as
        # single and steer away from the heavier half.
        self.declare_parameter("lane_gap_frac_min", 0.35)
        # Global sign multiplier on lane-keep line_err. +1.0 (default) means
        # L-only produces vy<0 (RIGHT on REP-103). Launch with
        # lane_keep_sign:=-1.0 if your robot's y-axis is inverted and the
        # robot steers toward the visible line instead of away.
        self.declare_parameter("lane_keep_sign", 1.0)
        self.declare_parameter("min_color_area_px", 400)

        # mission-level
        self.declare_parameter("to_intersection_m", 0.40)
        self.declare_parameter("dwell_seconds", 3.0)

        # safety
        self.declare_parameter("step_timeout_s", 90.0)
        self.declare_parameter("step_max_distance_m", 3.5)

        # debug
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "mission_runner")

        # cache
        g = self.get_parameter
        self.image_topic = str(g("image_topic").value)
        self.cmd_topic = str(g("cmd_topic").value)
        self.odom_topic = str(g("odom_topic").value)
        self.debug_topic = str(g("debug_image_topic").value)
        self.state_topic = str(g("state_topic").value)
        self.forward_speed = float(g("forward_speed").value)
        self.approach_speed = float(g("approach_speed").value)
        self.coast_speed = float(g("coast_speed").value)
        self.max_lat = float(g("max_lateral_speed").value)
        self.max_ang = float(g("max_angular_speed").value)
        self.rate_hz = float(g("control_rate_hz").value)
        self.line_kp = float(g("line_kp").value)
        self.line_kd = float(g("line_kd").value)
        self.err_sign = float(g("error_sign").value)
        self.yaw_kp = float(g("yaw_kp").value)
        self.yaw_tol = float(g("yaw_tolerance_rad").value)
        self.turn_sign = float(g("turn_sign").value)
        self.roi_top = float(g("roi_top_frac").value)
        self.roi_bot = float(g("roi_bot_frac").value)
        self.black_thresh = int(g("black_thresh").value)
        self.min_line_area = float(g("min_line_area_px").value)
        self.single_line_offset = float(g("single_line_offset").value)
        self.lane_gap_frac_min = float(g("lane_gap_frac_min").value)
        self.lane_keep_sign = float(g("lane_keep_sign").value)
        self.min_color_area = float(g("min_color_area_px").value)
        self.to_intersection = float(g("to_intersection_m").value)
        self.dwell_seconds_p = float(g("dwell_seconds").value)
        self.step_timeout = float(g("step_timeout_s").value)
        self.step_max_dist = float(g("step_max_distance_m").value)
        self.show_window = bool(g("show_window").value)
        self.window_name = str(g("window_name").value)

        # shared vision/odom state (updated from callbacks)
        self._line_found = False
        self._line_err = 0.0
        self._lane_mode = "none"
        self._active_color = None
        self._color_found = False
        self._color_err = 0.0
        self._color_frac = 0.0
        self._pose = None                # (x, y, yaw)
        self._img_received = False
        self._debug_frame = None

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 1)
        self.state_pub = self.create_publisher(String, self.state_topic, 1)
        self.create_subscription(Image, self.image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom,
                                 qos_profile_sensor_data)

        if self.show_window:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 640, 480)
            except Exception as e:
                self.get_logger().warn(f"cv2 window init failed ({e}); disabling.")
                self.show_window = False

        # mission state
        self.mission = self.build_mission()
        self.step_idx = -1                 # -1 = not started; will enter 0 on first tick
        self.step_state = {}
        self.mission_done = False

        self.control_timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"mission_runner up: image={self.image_topic} odom={self.odom_topic} "
            f"steps={len(self.mission)}"
        )

    # ------------------------------------------------------------------
    # Mission definition. Edit here to change the course.
    # ------------------------------------------------------------------
    def build_mission(self):
        # ================================================================
        # MISSION DEFINITION — tune values directly in the step dicts below.
        #
        # Conventions:
        #   distance        meters. STRAIGHT: + = forward, - = reverse.
        #                           STRAFE:   + = LEFT (+y), - = RIGHT (-y).
        #   dtheta          radians (deg(...) helper). + = LEFT / CCW per REP-103.
        #   follow_line     True = line-follow PID, False = open-loop drive.
        #
        # Per-step SPEED overrides (all optional; omit to use node defaults):
        #   STRAIGHT  "speed"          — m/s along vx   (default forward_speed)
        #             "coast_speed"    — m/s when line lost (default coast_speed)
        #   STRAFE    "speed"          — m/s along vy   (default forward_speed)
        #   TURN      "angular_speed"  — rad/s cap on wz (default max_angular_speed)
        #   COLOR     — no motion tunables; holds still until detected.
        #
        # Destination shape:
        #   pre-move -> COLOR (detect) -> STRAIGHT + (approach) -> DWELL ->
        #   STRAIGHT - (retreat)
        # ================================================================
        deg = math.radians
        return [
            # Leg 1: start -> yellow (west).
            {"name": "L1_south",        "kind": STEP_STRAIGHT, "distance": 0.5,
             "speed": 0.1,            "follow_line": False},
            {"name": "L1_face_west",    "kind": STEP_TURN,     "dtheta": deg(+90.0),
             "angular_speed": 0.4},
            {"name": "L1_find_yellow",  "kind": STEP_COLOR,    "target": "yellow"},
            {"name": "L1_approach",     "kind": STEP_STRAIGHT, "distance": 0.6,
             "speed": 0.1,            "follow_line": True},
            {"name": "L1_dwell",        "kind": STEP_DWELL,    "seconds": self.dwell_seconds_p},
            {"name": "L1_retreat",      "kind": STEP_STRAIGHT, "distance": -0.5,
             "speed": 0.1,            "follow_line": True},

            # Leg 2: yellow -> blue (north).
            {"name": "L2_face_south",   "kind": STEP_TURN,     "dtheta": deg(-90.0),
             "angular_speed": 0.4},
            {"name": "L2_strafe",       "kind": STEP_STRAFE,   "distance": -0.60,
             "speed": 0.1},
            {"name": "L2_find_blue",    "kind": STEP_COLOR,    "target": "blue"},
            {"name": "L2_approach",     "kind": STEP_STRAIGHT, "distance": 0.5,
             "speed": 0.1,            "follow_line": True},
            {"name": "L2_dwell",        "kind": STEP_DWELL,    "seconds": self.dwell_seconds_p},
            {"name": "L2_retreat",      "kind": STEP_STRAIGHT, "distance": -0.5,
             "speed": 0.1,            "follow_line": True},

            # Leg 3: blue -> green (east).
            {"name": "L3_face_east",      "kind": STEP_TURN,     "dtheta": deg(-90.0),
             "angular_speed": 0.4},
            {"name": "L3_east",        "kind": STEP_STRAIGHT, "distance": 0.40,
             "speed": 0.1,            "follow_line": True},
            {"name": "L3_find_green",   "kind": STEP_COLOR,    "target": "green"},
            {"name": "L3_approach",     "kind": STEP_STRAIGHT, "distance": 0.6,
             "speed": 0.1,            "follow_line": True},
            {"name": "L3_dwell",        "kind": STEP_DWELL,    "seconds": self.dwell_seconds_p},
        ]

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_image(self, msg: Image):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return
        self._last_cv_frame = cv

        h, w = cv.shape[:2]
        y1 = clamp(int(h * self.roi_top), 0, h)
        y2 = clamp(int(h * self.roi_bot), 0, h)
        if y2 <= y1:
            return
        roi = cv[y1:y2]

        hsv_full = cv2.cvtColor(cv, cv2.COLOR_BGR2HSV)

        # ----------------------------------------------------------------
        # LINE DETECTION — grayscale intensity thresholding.
        # Not color (HSV) and not contour-based. Pipeline per frame:
        #   1. BGR -> grayscale
        #   2. Inverse binary threshold: pixels with gray < black_thresh
        #      become 255 (white = "line"), everything else 0.
        #   3. Erode(1) then dilate(2) with a 3x3 kernel — removes salt-noise
        #      specks, then re-thickens the line so moments are stable.
        # Centroids below come from cv2.moments on the binary mask (image
        # moments of a 0/255 blob), NOT cv2.findContours. Area gating uses
        # cv2.countNonZero (actual pixel count) because moments m00 on a
        # 0/255 mask is 255*pixel_count, which makes direct comparison
        # against a pixel threshold wrong by a factor of 255.
        # ----------------------------------------------------------------
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, black = cv2.threshold(gray, self.black_thresh, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((3, 3), np.uint8)
        black = cv2.erode(black, kernel, iterations=1)
        black = cv2.dilate(black, kernel, iterations=2)

        # Lane-keeping line detection. Split the ROI mask into left and right
        # halves. If both halves have a line, centre on the midpoint (true
        # road centre). If only one side is visible, synthesize a negative
        # error so the PID steers AWAY from it — fixes the old bug where
        # the robot would chase a single visible line and drift onto it.
        half_w = w // 2
        left_mask = black[:, :half_w]
        right_mask = black[:, half_w:]
        # Use countNonZero for the area gate (actual pixel count). cv2.moments
        # m00 on a 0/255 binary mask is 255 * pixel_count, so comparing m00
        # against min_line_area_px without the 255 factor effectively accepts
        # any >1 pixel blob -> noisy centroid.
        left_pixels = cv2.countNonZero(left_mask)
        right_pixels = cv2.countNonZero(right_mask)
        half_thresh = self.min_line_area * 0.5
        left_found = left_pixels > half_thresh
        right_found = right_pixels > half_thresh
        Ml = cv2.moments(left_mask) if left_found else None
        Mr = cv2.moments(right_mask) if right_found else None
        left_cx = (Ml["m10"] / Ml["m00"]) if left_found else None
        left_cy = (Ml["m01"] / Ml["m00"]) if left_found else None
        right_cx = (Mr["m10"] / Mr["m00"] + half_w) if right_found else None
        right_cy = (Mr["m01"] / Mr["m00"]) if right_found else None

        line_cx = line_cy = None
        line_found = left_found or right_found
        line_err = 0.0
        # If both halves fire but the centroids sit close together, it's
        # almost certainly one line straddling the split. Demote to
        # single-line on whichever half has more pixels.
        min_gap_px = self.lane_gap_frac_min * w
        dual = (left_found and right_found and
                (right_cx - left_cx) >= min_gap_px)
        self._lane_mode = "none"
        if dual:
            line_cx = 0.5 * (left_cx + right_cx)
            line_cy = 0.5 * (left_cy + right_cy)
            line_err = (line_cx - w / 2.0) / (w / 2.0)
            self._lane_mode = "dual"
        elif left_found and right_found:
            # Gap too small -> single line straddling. Pick dominant half.
            if left_pixels >= right_pixels:
                line_cx = left_cx
                line_cy = left_cy
                line_err = +self.single_line_offset
                self._lane_mode = "L-straddle"
            else:
                line_cx = right_cx
                line_cy = right_cy
                line_err = -self.single_line_offset
                self._lane_mode = "R-straddle"
        elif left_found:
            line_cx = left_cx
            line_cy = left_cy
            # Only left edge visible -> drift to the right. line_err>0 means
            # "blob is in the right half" in the original convention, which
            # the downstream PID (err_sign=-1) translates into vy<0 = RIGHT.
            line_err = +self.single_line_offset
            self._lane_mode = "L-only"
        elif right_found:
            line_cx = right_cx
            line_cy = right_cy
            line_err = -self.single_line_offset
            self._lane_mode = "R-only"
        # Global sign override for robots whose y-axis is inverted.
        line_err *= self.lane_keep_sign

        # color mask (if a target is active)
        target_name = self._active_color
        target_cx = target_cy = None
        target_err = 0.0
        target_found = False
        target_frac = 0.0
        target_mask = None
        if target_name is not None:
            target_mask = mask_for_color(hsv_full, target_name)
            target_pixels = cv2.countNonZero(target_mask)
            target_frac = float(target_pixels) / float(h * w)
            if target_pixels > self.min_color_area:
                Mc = cv2.moments(target_mask)
                target_cx = Mc["m10"] / Mc["m00"]
                target_cy = Mc["m01"] / Mc["m00"]
                target_err = (target_cx - w / 2.0) / (w / 2.0)
                target_found = True

        self._img_received = True
        self._line_found = line_found
        self._line_err = line_err
        self._color_found = target_found
        self._color_err = target_err
        self._color_frac = target_frac

        # debug overlay
        want_debug = self.show_window or self.debug_pub.get_subscription_count() > 0
        if not want_debug:
            return
        dbg = cv.copy()
        overlay = cv2.cvtColor(black, cv2.COLOR_GRAY2BGR)
        dbg[y1:y2] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
        if target_mask is not None:
            tint = np.zeros_like(dbg)
            tint[:, :, 1] = target_mask
            dbg = cv2.addWeighted(dbg, 1.0, tint, 0.5, 0)
        cv2.line(dbg, (0, y1), (w, y1), (0, 200, 200), 1)
        cv2.line(dbg, (0, y2 - 1), (w, y2 - 1), (0, 200, 200), 1)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        # Draw L / R centroids separately so lane-keeping behavior is visible.
        if left_cx is not None:
            cv2.circle(dbg, (int(left_cx), int(left_cy + y1)), 5, (0, 200, 255), -1)
            cv2.putText(dbg, "L", (int(left_cx) - 18, int(left_cy + y1) + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        if right_cx is not None:
            cv2.circle(dbg, (int(right_cx), int(right_cy + y1)), 5, (0, 200, 255), -1)
            cv2.putText(dbg, "R", (int(right_cx) + 8, int(right_cy + y1) + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        if line_cx is not None:
            cv2.circle(dbg, (int(line_cx), int(line_cy + y1)), 7, (255, 255, 0), 2)
        if target_cx is not None:
            cv2.circle(dbg, (int(target_cx), int(target_cy)), 10, (0, 255, 0), 2)
        state_label = self._current_step_name() or "IDLE"
        cv2.putText(
            dbg,
            f"[{state_label}] target={target_name} frac={target_frac:.2f} "
            f"found={target_found} lane={self._lane_mode} err={line_err:+.2f}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
        )
        if self.debug_pub.get_subscription_count() > 0:
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
            except Exception:
                pass
        self._debug_frame = dbg

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------
    def _publish(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self._publish()

    def _publish_state(self, text):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _ready(self):
        return self._img_received and (self._pose is not None)

    def _current_step_name(self):
        if self.step_idx < 0 or self.step_idx >= len(self.mission):
            return None
        return self.mission[self.step_idx]["name"]

    def _show_debug(self):
        if not self.show_window or self._debug_frame is None:
            return
        try:
            cv2.imshow(self.window_name, self._debug_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"cv2.imshow failed ({e}); disabling.")
            self.show_window = False

    # ------------------------------------------------------------------
    # State machine tick
    # ------------------------------------------------------------------
    def _tick(self):
        self._show_debug()

        if self.mission_done:
            self._publish_stop()
            return

        if not self._ready():
            return

        # Advance to step 0 on first ready tick.
        if self.step_idx < 0:
            self.step_idx = 0
            self._enter_step()
            return

        if self.step_idx >= len(self.mission):
            self.mission_done = True
            self._publish_stop()
            self._publish_state("DONE")
            self.get_logger().info("=== MISSION COMPLETE ===")
            return

        step = self.mission[self.step_idx]
        try:
            done = self._dispatch_tick(step)
        except Exception as e:
            self.get_logger().error(f"step '{step['name']}' raised: {e}")
            self._publish_stop()
            self.mission_done = True
            return

        # periodic progress log (~1 Hz)
        t_elapsed = time.monotonic() - self.step_state.get("start_t", time.monotonic())
        last_log = self.step_state.get("last_log_t", 0.0)
        if t_elapsed - last_log >= 1.0:
            self.step_state["last_log_t"] = t_elapsed
            self._log_progress(step, t_elapsed)

        # global per-step timeout
        if t_elapsed > self.step_timeout:
            self.get_logger().error(
                f"step '{step['name']}' timeout after {t_elapsed:.1f}s")
            self._publish_stop()
            self.mission_done = True
            return

        if done:
            self._publish_stop()
            self.get_logger().info(
                f"[{self.step_idx}] '{step['name']}' DONE "
                f"({t_elapsed:.1f}s)")
            self.step_idx += 1
            self._enter_step()

    def _enter_step(self):
        self.step_state = {
            "start_t": time.monotonic(),
            "last_t": None,
            "prev_err": 0.0,
            "start_pose": self._pose,
            "start_yaw": self._pose[2] if self._pose else 0.0,
        }
        if self.step_idx >= len(self.mission):
            return
        step = self.mission[self.step_idx]
        self._dispatch_enter(step)
        self._publish_state(f"{self.step_idx}:{step['name']}:{step['kind']}")

    def _dispatch_enter(self, step):
        # Per-kind entry hook. Subclasses override to add new step kinds;
        # call super()._dispatch_enter(step) first to keep base kinds working.
        kind = step["kind"]
        name = step["name"]
        pose = self._pose
        if kind == STEP_TURN:
            dtheta_eff = self.turn_sign * step["dtheta"]
            target = wrap_angle(pose[2] + dtheta_eff)
            self.step_state["target_yaw"] = target
            self.get_logger().info(
                f"[{self.step_idx}] -> '{name}' TURN dtheta={math.degrees(dtheta_eff):+.1f}deg "
                f"(sign={self.turn_sign:+.0f}) "
                f"| yaw {math.degrees(pose[2]):+.1f}deg -> "
                f"target {math.degrees(target):+.1f}deg")
        elif kind == STEP_STRAIGHT:
            signed = float(step["distance"])
            direction = "fwd" if signed >= 0.0 else "rev"
            self.get_logger().info(
                f"[{self.step_idx}] -> '{name}' STRAIGHT {direction} "
                f"{abs(signed):.2f}m follow_line={step.get('follow_line', True)}")
        elif kind == STEP_STRAFE:
            signed = float(step["distance"])
            direction = "left" if signed >= 0.0 else "right"
            speed = float(step.get("speed", self.forward_speed))
            self.get_logger().info(
                f"[{self.step_idx}] -> '{name}' STRAFE {direction} "
                f"{abs(signed):.2f}m @ {speed:.3f}m/s")
        elif kind == STEP_COLOR:
            self._active_color = step["target"]
            self.get_logger().info(
                f"[{self.step_idx}] -> '{name}' COLOR target={step['target']}")
        elif kind == STEP_DWELL:
            self.get_logger().info(
                f"[{self.step_idx}] -> '{name}' DWELL {step['seconds']:.1f}s")

    def _dispatch_tick(self, step) -> bool:
        # Per-kind tick hook. Subclasses override to add new step kinds;
        # call super()._dispatch_tick(step) as the fallback for unhandled kinds.
        kind = step["kind"]
        if kind == STEP_STRAIGHT:
            return self._run_straight(step)
        if kind == STEP_STRAFE:
            return self._run_strafe(step)
        if kind == STEP_TURN:
            return self._run_turn(step)
        if kind == STEP_COLOR:
            return self._run_color(step)
        if kind == STEP_DWELL:
            return self._run_dwell(step)
        self.get_logger().error(f"unknown step kind: {kind}")
        return True

    # ------------------------------------------------------------------
    # Per-step tick logic
    # ------------------------------------------------------------------
    def _dt(self):
        now = time.monotonic()
        last = self.step_state.get("last_t")
        self.step_state["last_t"] = now
        if last is None:
            return 1.0 / self.rate_hz
        return max(1e-3, now - last)

    def _run_straight(self, step):
        # Signed distance: + = forward (+x), - = reverse (-x). Line PID runs
        # in both directions because +y is always LEFT in base_link regardless
        # of vx sign, and the camera keeps seeing the line ahead.
        start_pose = self.step_state["start_pose"]
        if start_pose is None:
            return True
        traveled = math.hypot(self._pose[0] - start_pose[0],
                              self._pose[1] - start_pose[1])
        signed = float(step["distance"])
        if traveled >= abs(signed):
            return True

        reverse = signed < 0.0
        # Per-step speed override: step["speed"] sets both the active drive
        # speed and (when the line is lost and we coast) the coast speed.
        # Defaults fall back to the node-level forward_speed / coast_speed.
        speed = float(step.get("speed", self.forward_speed))
        coast_speed = float(step.get("coast_speed", self.coast_speed))
        base_vx = -speed if reverse else speed
        coast_vx = -coast_speed if reverse else coast_speed

        # open-loop: no line tracking, just drive.
        if not step.get("follow_line", True):
            self._publish(vx=base_vx)
            return False

        # Lane-keep:
        #   L-only / L-straddle -> bang-bang RIGHT (vy = -max_lat)
        #   R-only / R-straddle -> bang-bang LEFT  (vy = +max_lat)
        #   dual                -> PID fine-centering
        #   none                -> coast with no lateral
        # lane_keep_sign flips the sign for robots with inverted y-axis.
        mode = self._lane_mode
        if mode in ("L-only", "L-straddle"):
            vy = -self.max_lat * self.lane_keep_sign
            self.step_state["prev_err"] = 0.0
        elif mode in ("R-only", "R-straddle"):
            vy = +self.max_lat * self.lane_keep_sign
            self.step_state["prev_err"] = 0.0
        elif mode == "dual":
            dt = self._dt()
            err = self.err_sign * self._line_err
            deriv = (err - self.step_state["prev_err"]) / dt
            self.step_state["prev_err"] = err
            vy = clamp(self.line_kp * err + self.line_kd * deriv,
                       -self.max_lat, self.max_lat)
        else:
            self.step_state["prev_err"] = 0.0
            self._publish(vx=coast_vx)
            return False

        self._publish(vx=base_vx, vy=vy)
        return False

    def _run_strafe(self, step):
        # Pure lateral motion. No vx, no PID, no line-follow. Direction
        # encodes the sign of vy: "left" -> +y, "right" -> -y (base_link
        # convention: +y=LEFT per ROS REP-103). Terminates on odom distance.
        start_pose = self.step_state["start_pose"]
        if start_pose is None:
            return True
        traveled = math.hypot(self._pose[0] - start_pose[0],
                              self._pose[1] - start_pose[1])
        signed = float(step["distance"])
        if traveled >= abs(signed):
            return True
        speed = float(step.get("speed", self.forward_speed))
        # Sign of distance sets lateral direction: + -> +y (LEFT), - -> -y (RIGHT).
        vy = speed if signed >= 0.0 else -speed
        self._publish(vy=vy)
        return False

    def _run_turn(self, step):
        target = self.step_state["target_yaw"]
        err = wrap_angle(target - self._pose[2])
        if abs(err) < self.yaw_tol:
            self.get_logger().info(
                f"    turn done | final_yaw={math.degrees(self._pose[2]):+.1f}deg "
                f"(residual {math.degrees(err):+.2f}deg)")
            return True
        # Per-step angular speed cap: step["angular_speed"] in rad/s. Falls
        # back to the node-level max_angular_speed.
        max_wz = float(step.get("angular_speed", self.max_ang))
        wz = clamp(self.yaw_kp * err, -max_wz, max_wz)
        self._publish(wz=wz)
        return False

    def _run_color(self, step):
        # Detect-and-trigger. The step holds the robot still until the HSV
        # detector in _on_image flips self._color_found True, then advances.
        # Assumes the preceding TURN/STRAFE already points the camera at the
        # swatch, so the color is visible on entry.
        if self._color_found:
            self.get_logger().info(
                f"    color '{step['target']}' DETECTED "
                f"(frac={self._color_frac:.2f})")
            self._active_color = None
            return True

        self._publish_stop()
        return False

    def _log_progress(self, step, elapsed):
        kind = step["kind"]
        name = step["name"]
        if kind == STEP_STRAIGHT:
            sp = self.step_state.get("start_pose")
            if sp is not None and self._pose is not None:
                traveled = math.hypot(self._pose[0] - sp[0], self._pose[1] - sp[1])
                signed = float(step["distance"])
                direction = "fwd" if signed >= 0.0 else "rev"
                self.get_logger().info(
                    f"  [{self.step_idx}] '{name}' STRAIGHT {direction} "
                    f"t={elapsed:.1f}s traveled={traveled:.2f}/{abs(signed):.2f}m "
                    f"lane={self._lane_mode} err={self._line_err:+.2f} "
                    f"yaw={math.degrees(self._pose[2]):+.1f}deg")
        elif kind == STEP_STRAFE:
            sp = self.step_state.get("start_pose")
            if sp is not None and self._pose is not None:
                traveled = math.hypot(self._pose[0] - sp[0], self._pose[1] - sp[1])
                signed = float(step["distance"])
                direction = "left" if signed >= 0.0 else "right"
                self.get_logger().info(
                    f"  [{self.step_idx}] '{name}' STRAFE {direction} "
                    f"t={elapsed:.1f}s traveled={traveled:.2f}/{abs(signed):.2f}m")
        elif kind == STEP_TURN:
            target = self.step_state.get("target_yaw", 0.0)
            err = wrap_angle(target - self._pose[2]) if self._pose else 0.0
            self.get_logger().info(
                f"  [{self.step_idx}] '{name}' TURN "
                f"t={elapsed:.1f}s yaw={math.degrees(self._pose[2]):+.1f}deg "
                f"target={math.degrees(wrap_angle(target)):+.1f}deg "
                f"err={math.degrees(err):+.1f}deg")
        elif kind == STEP_COLOR:
            self.get_logger().info(
                f"  [{self.step_idx}] '{name}' COLOR target={step['target']} "
                f"t={elapsed:.1f}s frac={self._color_frac:.3f} "
                f"color_found={self._color_found} line={self._line_found}")
        elif kind == STEP_DWELL:
            self.get_logger().info(
                f"  [{self.step_idx}] '{name}' DWELL t={elapsed:.1f}/{step['seconds']:.1f}s")

    def _run_dwell(self, step):
        elapsed = time.monotonic() - self.step_state["start_t"]
        self._publish_stop()
        return elapsed >= step["seconds"]


def main():
    rclpy.init()
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_stop()
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
