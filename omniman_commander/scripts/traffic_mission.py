#!/usr/bin/env python3
# Traffic-order mission runner (extends mission_runner.MissionRunner).
#
# Flow:
#   1. Drive forward to the traffic panel (STRAIGHT).
#   2. Raise the arm to the SRDF "traffic" named pose (ARM_POSE).
#   3. Detect the left-to-right color order of the 3 swatches on the panel
#      (DETECT_ORDER, HSV-based, majority vote over N frames).
#   4. Lower the arm back to the "pid" pose so the palm camera faces the
#      floor for line-following (ARM_POSE).
#   5. Splice hub-and-spoke legs into the mission in detected order
#      (BUILD_LEGS). Each spoke is: hub -> color -> VERIFY_COLOR -> dwell
#      -> hub. If VERIFY fails the approach+dwell+retreat are skipped via
#      the "skip_if" flag and the robot returns to hub unchanged.
#
# New step kinds added to the base state machine:
#   ARM_POSE       — send FollowJointTrajectory to /arm_controller, wait
#                    until result future fires. step["pose"] = "traffic" | "pid".
#   DETECT_ORDER   — sample N frames from /image_raw, sort HSV centroids L->R,
#                    commit the majority order into self._traffic_order.
#   BUILD_LEGS     — one-shot: splice _spoke(color) for each color in
#                    self._traffic_order after the current step_idx.
#   VERIFY_COLOR   — wait up to timeout_s for self._color_found. On pass,
#                    proceed. On fail, add step["fail_flag"] to self._skip_flags
#                    so subsequent steps with matching "skip_if" auto-complete.

import math
import time
from typing import List, Optional

import cv2
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectoryPoint

from mission_runner import (
    MissionRunner,
    STEP_STRAIGHT, STEP_TURN, STEP_DWELL,
    mask_for_color,
)


STEP_ARM_POSE = "ARM_POSE"
STEP_DETECT_ORDER = "DETECT_ORDER"
STEP_BUILD_LEGS = "BUILD_LEGS"
STEP_VERIFY_COLOR = "VERIFY_COLOR"


# Must match the joint order under arm_controller in controllers.yaml.
ARM_JOINT_ORDER = [
    "shoulder_yaw_joint",
    "upper_shoulder_pitch_joint",
    "arm_yaw_joint",
    "forearm_pitch_joint",
    "wrist_pitch_joint",
    "palm_yaw_joint",
]

# Joint values copied from omniman_moveit_config/config/nxp_omniman.srdf
# group_state blocks. Keep in the joint order above.
ARM_POSES = {
    "pid":     [0.0,  0.1745, 0.0, -1.0472, 1.6929, 3.1415],
    "traffic": [0.0, -1.6929, 0.0, -0.1745, 0.3490, 3.1415],
}


class TrafficMissionRunner(MissionRunner):
    def __init__(self):
        super().__init__()

        self.declare_parameter("arm_action",
                               "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("arm_move_time_s", 3.0)
        self.declare_parameter("detect_frames", 12)
        self.declare_parameter("detect_min_blob_px", 400)
        self.declare_parameter("traffic_approach_m", 0.2)
        self.declare_parameter("post_detect_m", 0.3)

        self.arm_action_name = str(self.get_parameter("arm_action").value)
        self.arm_move_time = float(self.get_parameter("arm_move_time_s").value)
        self.detect_frames_target = int(
            self.get_parameter("detect_frames").value)
        self.detect_min_blob = int(
            self.get_parameter("detect_min_blob_px").value)
        self.traffic_approach = float(
            self.get_parameter("traffic_approach_m").value)
        self.post_detect = float(self.get_parameter("post_detect_m").value)

        self._arm_client = ActionClient(self, FollowJointTrajectory,
                                        self.arm_action_name)
        self._arm_sent = False
        self._arm_success = False

        self._traffic_order: List[str] = []
        self._detect_samples: List[List[str]] = []
        self._detect_last_t = 0.0

        self._skip_flags: set = set()
        self._legs_built = False

        # Parent already built its own (fixed Y-B-G) mission during
        # super().__init__() — rebuild with our static prefix now that all
        # subclass attributes exist.
        self.mission = self.build_mission()
        self.step_idx = -1

    # ------------------------------------------------------------------
    # Static prefix + per-color spoke tables.
    #
    # Prefix runs once at mission start. TO_PANEL is capped at ~0.2 m so
    # the stowed arm does not collide with the panel during detection;
    # POST_DETECT_FWD closes the remaining gap (~0.3 m) after the arm is
    # back in the "pid" pose, so by the time BUILD_LEGS runs the robot is
    # sitting at the hub just like mission_runner.py (≈0.5 m forward).
    #
    # Spokes live in self._spokes keyed by color. Every value is visible
    # here so you can tune distances/speeds without reading dispatch code.
    # Each spoke MUST end at the same (x, y, yaw) it started — otherwise
    # later spokes will drift.
    # ------------------------------------------------------------------
    def build_mission(self):
        approach = getattr(self, "traffic_approach", 0.6)
        post_fwd = getattr(self, "post_detect", 0.3)
        dwell_s = self.dwell_seconds_p

        self._spokes = {
            "yellow": [
                {"name": "yellow_turn_out", "kind": STEP_TURN,
                 "dtheta": math.radians(+90.0), "angular_speed": 0.2},
                {"name": "yellow_out",      "kind": STEP_STRAIGHT,
                 "distance": 0.30, "speed": 0.10, "follow_line": True},
                {"name": "yellow_verify",   "kind": STEP_VERIFY_COLOR,
                 "target": "yellow", "timeout_s": 2.5,
                 "fail_flag": "skip_yellow"},
                {"name": "yellow_approach", "kind": STEP_STRAIGHT,
                 "distance": 0.2, "speed": 0.06, "follow_line": True,
                 "skip_if": "skip_yellow"},
                {"name": "yellow_dwell",    "kind": STEP_DWELL,
                 "seconds": dwell_s, "skip_if": "skip_yellow"},
                {"name": "yellow_retreat",  "kind": STEP_STRAIGHT,
                 "distance": -0.2, "speed": 0.06, "follow_line": True,
                 "skip_if": "skip_yellow"},
                {"name": "yellow_back",     "kind": STEP_STRAIGHT,
                 "distance": -0.50, "speed": 0.10, "follow_line": True},
                {"name": "yellow_turn_back","kind": STEP_TURN,
                 "dtheta": math.radians(-90.0), "angular_speed": 0.2},
            ],
            "blue": [
                # blue sits straight ahead of the hub, no turn needed.
                {"name": "blue_out",      "kind": STEP_STRAIGHT,
                 "distance": 0.50, "speed": 0.10, "follow_line": True},
                {"name": "blue_verify",   "kind": STEP_VERIFY_COLOR,
                 "target": "blue", "timeout_s": 2.5,
                 "fail_flag": "skip_blue"},
                {"name": "blue_approach", "kind": STEP_STRAIGHT,
                 "distance": 0.35, "speed": 0.06, "follow_line": True,
                 "skip_if": "skip_blue"},
                {"name": "blue_dwell",    "kind": STEP_DWELL,
                 "seconds": dwell_s, "skip_if": "skip_blue"},
                {"name": "blue_retreat",  "kind": STEP_STRAIGHT,
                 "distance": -0.35, "speed": 0.06, "follow_line": True,
                 "skip_if": "skip_blue"},
                {"name": "blue_back",     "kind": STEP_STRAIGHT,
                 "distance": -0.50, "speed": 0.10, "follow_line": True},
            ],
            "green": [
                {"name": "green_turn_out", "kind": STEP_TURN,
                 "dtheta": math.radians(-90.0), "angular_speed": 0.2},
                {"name": "green_out",      "kind": STEP_STRAIGHT,
                 "distance": 0.50, "speed": 0.10, "follow_line": True},
                {"name": "green_verify",   "kind": STEP_VERIFY_COLOR,
                 "target": "green", "timeout_s": 2.5,
                 "fail_flag": "skip_green"},
                {"name": "green_approach", "kind": STEP_STRAIGHT,
                 "distance": 0.35, "speed": 0.06, "follow_line": True,
                 "skip_if": "skip_green"},
                {"name": "green_dwell",    "kind": STEP_DWELL,
                 "seconds": dwell_s, "skip_if": "skip_green"},
                {"name": "green_retreat",  "kind": STEP_STRAIGHT,
                 "distance": -0.35, "speed": 0.06, "follow_line": True,
                 "skip_if": "skip_green"},
                {"name": "green_back",     "kind": STEP_STRAIGHT,
                 "distance": -0.50, "speed": 0.10, "follow_line": True},
                {"name": "green_turn_back","kind": STEP_TURN,
                 "dtheta": math.radians(+90.0), "angular_speed": 0.2},
            ],
        }

        return [
            {"name": "TO_PANEL",        "kind": STEP_STRAIGHT,
             "distance": approach, "speed": 0.08, "follow_line": False},
            {"name": "ARM_TRAFFIC",     "kind": STEP_ARM_POSE,
             "pose": "traffic"},
            {"name": "DETECT",          "kind": STEP_DETECT_ORDER},
            {"name": "ARM_PID",         "kind": STEP_ARM_POSE,
             "pose": "pid"},
            {"name": "POST_DETECT_FWD", "kind": STEP_STRAIGHT,
             "distance": post_fwd, "speed": 0.08, "follow_line": False},
            {"name": "SPLICE",          "kind": STEP_BUILD_LEGS},
        ]

    # ------------------------------------------------------------------
    # Dispatch overrides — add new kinds + honor skip_if.
    # ------------------------------------------------------------------
    def _dispatch_enter(self, step):
        kind = step["kind"]
        if kind == STEP_ARM_POSE:
            self._arm_sent = False
            self._arm_success = False
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' ARM_POSE "
                f"pose={step['pose']}")
            return
        if kind == STEP_DETECT_ORDER:
            self._detect_samples = []
            self._detect_last_t = 0.0
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' DETECT_ORDER "
                f"(target {self.detect_frames_target} frames)")
            return
        if kind == STEP_BUILD_LEGS:
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' BUILD_LEGS "
                f"order={self._traffic_order}")
            return
        if kind == STEP_VERIFY_COLOR:
            self._active_color = step["target"]
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' VERIFY_COLOR "
                f"target={step['target']} "
                f"timeout={step.get('timeout_s', 2.0):.1f}s")
            return
        super()._dispatch_enter(step)

    def _dispatch_tick(self, step) -> bool:
        # Any step can carry "skip_if": <flag>. When the flag is in
        # self._skip_flags (set by a failed VERIFY_COLOR) the step
        # auto-completes. Used to short-circuit approach/dwell/retreat
        # when nav drops us at the wrong swatch.
        gate = step.get("skip_if")
        if gate and gate in self._skip_flags:
            self.get_logger().info(
                f"    skipping '{step['name']}' (flag '{gate}' set)")
            self._publish_stop()
            return True
        kind = step["kind"]
        if kind == STEP_ARM_POSE:
            return self._run_arm_pose(step)
        if kind == STEP_DETECT_ORDER:
            return self._run_detect_order(step)
        if kind == STEP_BUILD_LEGS:
            return self._run_build_legs(step)
        if kind == STEP_VERIFY_COLOR:
            return self._run_verify_color(step)
        return super()._dispatch_tick(step)

    # ------------------------------------------------------------------
    # ARM_POSE — FollowJointTrajectory action client
    # ------------------------------------------------------------------
    def _run_arm_pose(self, step) -> bool:
        pose_name = step["pose"]
        joints = ARM_POSES.get(pose_name)
        if joints is None:
            self.get_logger().error(f"unknown arm pose: {pose_name}")
            return True

        if not self._arm_sent:
            if not self._arm_client.wait_for_server(timeout_sec=0.1):
                self._publish_stop()
                return False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM_JOINT_ORDER
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in joints]
            pt.time_from_start = Duration(
                seconds=self.arm_move_time).to_msg()
            goal.trajectory.points.append(pt)
            future = self._arm_client.send_goal_async(goal)
            future.add_done_callback(self._arm_goal_response_cb)
            self._arm_sent = True
            self._publish_stop()
            return False

        self._publish_stop()
        return self._arm_success

    def _arm_goal_response_cb(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("    arm goal rejected")
            self._arm_success = True
            return
        handle.get_result_async().add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        wrapped = future.result()
        err = wrapped.result.error_code if wrapped is not None else -1
        if err == 0:
            self.get_logger().info("    arm goal SUCCEEDED")
        else:
            self.get_logger().error(
                f"    arm goal finished with error_code={err}")
        self._arm_success = True

    # ------------------------------------------------------------------
    # DETECT_ORDER — majority vote over N HSV-centroid samples
    # ------------------------------------------------------------------
    def _run_detect_order(self, step) -> bool:
        self._publish_stop()
        if len(self._detect_samples) >= self.detect_frames_target:
            order = self._commit_order()
            if order is None:
                self.get_logger().error(
                    "    detect: no consensus; falling back to Y-B-G")
                self._traffic_order = ["yellow", "blue", "green"]
            else:
                self._traffic_order = order
            self.get_logger().info(
                f"    traffic order COMMIT: {self._traffic_order}")
            return True

        now = time.monotonic()
        if now - self._detect_last_t < 0.15:
            return False
        self._detect_last_t = now

        sample = self._detect_sample_once()
        if sample is not None:
            self._detect_samples.append(sample)
            self.get_logger().info(
                f"    detect sample {len(self._detect_samples)}"
                f"/{self.detect_frames_target}: {sample}")
        return False

    def _detect_sample_once(self) -> Optional[List[str]]:
        cv = getattr(self, "_last_cv_frame", None)
        if cv is None:
            return None
        hsv = cv2.cvtColor(cv, cv2.COLOR_BGR2HSV)
        centroids = []
        for color in ("yellow", "blue", "green"):
            mask = mask_for_color(hsv, color)
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=2)
            if cv2.countNonZero(mask) < self.detect_min_blob:
                return None
            M = cv2.moments(mask)
            if M["m00"] <= 0:
                return None
            cx = M["m10"] / M["m00"]
            centroids.append((cx, color))
        centroids.sort(key=lambda t: t[0])
        return [c for _, c in centroids]

    def _commit_order(self) -> Optional[List[str]]:
        if not self._detect_samples:
            return None
        counts = {}
        for s in self._detect_samples:
            key = tuple(s)
            counts[key] = counts.get(key, 0) + 1
        best = max(counts.items(), key=lambda kv: kv[1])
        return list(best[0])

    # ------------------------------------------------------------------
    # BUILD_LEGS — splice spokes into the mission in detected order
    # ------------------------------------------------------------------
    def _run_build_legs(self, step) -> bool:
        if self._legs_built:
            return True
        if not self._traffic_order:
            self.get_logger().error(
                "    build_legs: no order committed; aborting")
            return True
        insert_at = self.step_idx + 1
        spokes = []
        for color in self._traffic_order:
            spokes.extend(self._spokes[color])
        self.mission[insert_at:insert_at] = spokes
        self._legs_built = True
        self.get_logger().info(
            f"    spliced {len(spokes)} steps for order "
            f"{self._traffic_order}")
        return True

    # ------------------------------------------------------------------
    # VERIFY_COLOR — pass if target visible, else raise skip flag
    # ------------------------------------------------------------------
    def _run_verify_color(self, step) -> bool:
        self._publish_stop()
        if self._color_found:
            self.get_logger().info(
                f"    verify '{step['target']}' PASS "
                f"(frac={self._color_frac:.2f})")
            self._active_color = None
            return True
        timeout = float(step.get("timeout_s", 2.0))
        elapsed = time.monotonic() - self.step_state["start_t"]
        if elapsed >= timeout:
            self._skip_flags.add(step["fail_flag"])
            self.get_logger().warn(
                f"    verify '{step['target']}' FAIL "
                f"(no color seen in {timeout:.1f}s) — skipping dwell")
            self._active_color = None
            return True
        return False


def main():
    rclpy.init()
    node = TrafficMissionRunner()
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
