#!/usr/bin/env python3
# Robosot pick-and-place mission orchestrator.
#
# Drives mecanum navigation (with line-PID where applicable) and delegates
# all arm operations to robosot_arm_server.cpp via /robosot/arm_cmd.
#
# Mission flow:
#   PHASE 1 — Initial nav to HOME:
#       gripper:open → arm:left → backward 0.3 → strafe right 0.5 →
#       backward 0.3.
#   PHASE 2 — Visit each goods area:
#       For yellow / blue / green spokes (same distances as traffic_mission):
#         - Drive home → spoke (PID line-follow on STRAIGHT legs)
#         - arm:north (search pose)
#         - scan:F,I,R,A → first found letter, or "none"
#         - If found:
#             pick:LETTER → arm:east → arm:left → drive spoke → home
#             arm:east → place:POSE_FOR_LETTER → arm:east
#             return to spoke
#         - Repeat scan until "none"
#         - arm:left → drive spoke → home
#       Repeat passes for any deferred letters.

import math
import time
from typing import List, Optional

import rclpy
from std_msgs.msg import String

from mission_runner import (
    MissionRunner,
    STEP_STRAIGHT, STEP_STRAFE, STEP_TURN, STEP_DWELL,
)


STEP_ARM_CMD = "ARM_CMD"          # send a string command, wait for "ok"/"fail"
STEP_SCAN    = "SCAN"             # scan letters, store result in self._last_scan
STEP_PICK    = "PICK"             # pick by letter from self._last_scan
STEP_PLACE   = "PLACE"            # place letter at place pose from self._last_scan
STEP_BUILD   = "BUILD_LEGS"       # splice the pick/place legs after a successful scan


# ---- Mission constants (tune to arena) ------------------------------------
INIT_PRE_BACKWARD_M  = 0.5
INIT_STRAFE_RIGHT_M  = 0.8   # strafe LEFT magnitude (sign applied at use site)
INIT_BACKWARD_M      = 0.3

LETTERS              = ["F", "I", "R", "A"]
PLACE_POSES          = {"F": "F1", "I": "I1", "R": "R1", "A": "A1"}

ARM_CMD_TIMEOUT_S    = 60.0       # wait this long for an arm command result
SCAN_TIMEOUT_S       = 30.0       # scan can be longer (4 letters × 5 s)


class RobosotMission(MissionRunner):
    def __init__(self):
        super().__init__()

        # Arm-server I/O.
        self._arm_cmd_pub = self.create_publisher(String, "/robosot/arm_cmd", 10)
        self.create_subscription(String, "/robosot/arm_result",
                                 self._on_arm_result, 10)
        self._arm_result: Optional[str] = None
        self._arm_sent_t: float = 0.0

        # Scan / multi-pass state.
        self._pending: List[str] = list(LETTERS)
        self._last_scan: Optional[str] = None
        self._spoke_idx: int = 0
        self._pass: int = 1
        self._max_passes: int = 3

        # Replace the parent's mission with our own now that all state exists.
        self.mission = self.build_mission()
        self.step_idx = -1

    # ------------------------------------------------------------------
    # Mission definition
    # ------------------------------------------------------------------
    def build_mission(self):
        deg = math.radians
        dwell = self.dwell_seconds_p

        # Per-spoke navigation legs (matches traffic_mission distances).
        # Each spoke has: name, prelude (turn or strafe), drive, retreat-back.
        self._spoke_legs = {
            "yellow": {
                "out": [
                    {"name": "yellow_turn_out", "kind": STEP_TURN,
                     "dtheta": deg(+100.0), "angular_speed": 0.2},
                    {"name": "yellow_drive",   "kind": STEP_STRAIGHT,
                     "distance": 0.40, "speed": 0.10, "follow_line": True},
                    {"name": "yellow_approach","kind": STEP_STRAIGHT,
                     "distance": 0.20, "speed": 0.06, "follow_line": True},
                ],
                "back": [
                    {"name": "yellow_retreat", "kind": STEP_STRAIGHT,
                     "distance": -0.20, "speed": 0.06, "follow_line": True},
                    {"name": "yellow_back",    "kind": STEP_STRAIGHT,
                     "distance": -0.40, "speed": 0.10, "follow_line": True},
                    {"name": "yellow_turn_back","kind": STEP_TURN,
                     "dtheta": deg(-100.0), "angular_speed": 0.2},
                ],
            },
            "blue": {
                "out": [
                    {"name": "blue_strafe_out", "kind": STEP_STRAFE,
                     "distance": -0.40, "speed": 0.08},
                    {"name": "blue_drive",      "kind": STEP_STRAIGHT,
                     "distance": 0.40, "speed": 0.10, "follow_line": True},
                    {"name": "blue_approach",   "kind": STEP_STRAIGHT,
                     "distance": 0.10, "speed": 0.06, "follow_line": True},
                ],
                "back": [
                    {"name": "blue_retreat",    "kind": STEP_STRAIGHT,
                     "distance": -0.10, "speed": 0.06, "follow_line": True},
                    {"name": "blue_back",       "kind": STEP_STRAIGHT,
                     "distance": -0.40, "speed": 0.10, "follow_line": True},
                    {"name": "blue_strafe_back","kind": STEP_STRAFE,
                     "distance": +0.40, "speed": 0.08},
                ],
            },
            "green": {
                "out": [
                    {"name": "green_turn_out", "kind": STEP_TURN,
                     "dtheta": deg(-90.0), "angular_speed": 0.2},
                    {"name": "green_drive",    "kind": STEP_STRAIGHT,
                     "distance": 1.0, "speed": 0.10, "follow_line": True},
                    {"name": "green_approach", "kind": STEP_STRAIGHT,
                     "distance": 0.35, "speed": 0.06, "follow_line": True},
                ],
                "back": [
                    {"name": "green_retreat",  "kind": STEP_STRAIGHT,
                     "distance": -0.35, "speed": 0.06, "follow_line": True},
                    {"name": "green_back",     "kind": STEP_STRAIGHT,
                     "distance": -1.0, "speed": 0.10, "follow_line": True},
                    {"name": "green_turn_back","kind": STEP_TURN,
                     "dtheta": deg(+90.0), "angular_speed": 0.2},
                ],
            },
        }

        # Top-level mission.
        # PHASE 2 (per-spoke visit + scan/pick/place loop) is built dynamically
        # by BUILD_LEGS step after each scan resolves to a letter.
        steps = [
            # ---- PHASE 1: start → HOME ----
            # Sequence: arm:north → backward → arm:left → strafe LEFT → backward.
            {"name": "PHASE1_open",      "kind": STEP_ARM_CMD, "cmd": "gripper:open"},
            {"name": "PHASE1_arm_north", "kind": STEP_ARM_CMD, "cmd": "named:north"},
            {"name": "PHASE1_back1",     "kind": STEP_STRAIGHT,
             "distance": -INIT_PRE_BACKWARD_M, "speed": 0.08, "follow_line": False},
            {"name": "PHASE1_arm_left",  "kind": STEP_ARM_CMD, "cmd": "named:left"},
            # +y = LEFT in REP-103.
            {"name": "PHASE1_strafe",    "kind": STEP_STRAFE,
             "distance": +INIT_STRAFE_RIGHT_M, "speed": 0.08},
            {"name": "PHASE1_back2",     "kind": STEP_STRAIGHT,
             "distance": -INIT_BACKWARD_M, "speed": 0.08, "follow_line": False},

            # ---- PHASE 2: visit yellow → blue → green ----
            *self._visit_spoke_steps("yellow"),
            *self._visit_spoke_steps("blue"),
            *self._visit_spoke_steps("green"),
        ]
        return steps

    def _visit_spoke_steps(self, spoke_name: str):
        """Build the steps for visiting one goods area.

        Out-legs → arm:north → SCAN → (BUILD_LEGS splices pick/place
        legs back into this slot) → arm:left → back-legs.
        """
        legs = self._spoke_legs[spoke_name]
        return [
            {"name": f"{spoke_name}_visit_arm_left", "kind": STEP_ARM_CMD,
             "cmd": "named:left"},
            *legs["out"],
            {"name": f"{spoke_name}_arm_search",    "kind": STEP_ARM_CMD,
             "cmd": "named:north"},
            {"name": f"{spoke_name}_scan",          "kind": STEP_SCAN,
             "spoke": spoke_name},
            {"name": f"{spoke_name}_build",         "kind": STEP_BUILD,
             "spoke": spoke_name},
            {"name": f"{spoke_name}_visit_done",    "kind": STEP_ARM_CMD,
             "cmd": "named:left"},
            *legs["back"],
        ]

    def _pickplace_steps(self, letter: str, spoke_name: str):
        """Build the pick → home → place → return-to-spoke leg chain."""
        legs = self._spoke_legs[spoke_name]
        place_pose = PLACE_POSES[letter]
        return [
            # PICK at goods area (already at search pose).
            {"name": f"{letter}_pick",        "kind": STEP_ARM_CMD,
             "cmd": f"pick:{letter}"},
            {"name": f"{letter}_arm_east",    "kind": STEP_ARM_CMD,
             "cmd": "named:east"},
            {"name": f"{letter}_arm_left",    "kind": STEP_ARM_CMD,
             "cmd": "named:left"},
            # Navigate goods area → HOME.
            *legs["back"],
            # PLACE at home.
            {"name": f"{letter}_arm_east_h",  "kind": STEP_ARM_CMD,
             "cmd": "named:east"},
            {"name": f"{letter}_place",       "kind": STEP_ARM_CMD,
             "cmd": f"place:{place_pose}"},
            {"name": f"{letter}_arm_east_p",  "kind": STEP_ARM_CMD,
             "cmd": "named:east"},
            # Navigate HOME → goods area to scan again.
            {"name": f"{letter}_arm_left_b",  "kind": STEP_ARM_CMD,
             "cmd": "named:left"},
            *legs["out"],
            {"name": f"{letter}_arm_search",  "kind": STEP_ARM_CMD,
             "cmd": "named:north"},
            # Re-scan after placing.
            {"name": f"{letter}_rescan",      "kind": STEP_SCAN,
             "spoke": spoke_name},
            {"name": f"{letter}_rebuild",     "kind": STEP_BUILD,
             "spoke": spoke_name},
        ]

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------
    def _dispatch_enter(self, step):
        kind = step["kind"]
        if kind == STEP_ARM_CMD:
            self._send_arm_cmd(step["cmd"])
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' ARM_CMD `{step['cmd']}`")
            return
        if kind == STEP_SCAN:
            if not self._pending:
                self.get_logger().info(
                    f"[{self.step_idx}] -> '{step['name']}' SCAN skipped (no pending)")
                return
            cmd = "scan:" + ",".join(self._pending)
            self._send_arm_cmd(cmd)
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' SCAN `{cmd}`")
            return
        if kind == STEP_BUILD:
            self.get_logger().info(
                f"[{self.step_idx}] -> '{step['name']}' BUILD_LEGS "
                f"last_scan={self._last_scan} pending={self._pending}")
            return
        super()._dispatch_enter(step)

    def _dispatch_tick(self, step) -> bool:
        kind = step["kind"]
        if kind == STEP_ARM_CMD:
            return self._tick_arm_cmd(step, ARM_CMD_TIMEOUT_S)
        if kind == STEP_SCAN:
            return self._tick_scan(step)
        if kind == STEP_BUILD:
            return self._tick_build_legs(step)
        return super()._dispatch_tick(step)

    # ------------------------------------------------------------------
    # ARM_CMD — send + wait for "ok" / "fail"
    # ------------------------------------------------------------------
    def _send_arm_cmd(self, cmd: str):
        self._arm_result = None
        self._arm_sent_t = time.monotonic()
        msg = String()
        msg.data = cmd
        self._arm_cmd_pub.publish(msg)

    def _on_arm_result(self, msg: String):
        self._arm_result = msg.data
        self.get_logger().info(f"  arm_result: `{msg.data}`")

    def _tick_arm_cmd(self, step, timeout_s: float) -> bool:
        self._publish_stop()
        if self._arm_result is None:
            if time.monotonic() - self._arm_sent_t > timeout_s:
                self.get_logger().error(
                    f"  arm_cmd `{step['cmd']}` TIMEOUT after {timeout_s:.0f}s")
                return True
            return False
        # Got a reply.
        result = self._arm_result
        self._arm_result = None
        if result.startswith("ok"):
            return True
        self.get_logger().warn(f"  arm_cmd `{step['cmd']}` -> `{result}`")
        return True   # advance regardless; orchestration handles failure flags

    # ------------------------------------------------------------------
    # SCAN — same wait protocol as ARM_CMD; result like "ok:F" or "none"
    # ------------------------------------------------------------------
    def _tick_scan(self, step) -> bool:
        self._publish_stop()
        if not self._pending:
            self._last_scan = None
            return True
        if self._arm_result is None:
            if time.monotonic() - self._arm_sent_t > SCAN_TIMEOUT_S:
                self.get_logger().error("  SCAN TIMEOUT")
                self._last_scan = None
                return True
            return False
        result = self._arm_result
        self._arm_result = None
        if result.startswith("ok:"):
            self._last_scan = result[3:].strip()
            self.get_logger().info(f"  SCAN found `{self._last_scan}`")
        else:
            self._last_scan = None
            self.get_logger().info("  SCAN found nothing")
        return True

    # ------------------------------------------------------------------
    # BUILD_LEGS — splice pick/place steps after a successful scan
    # ------------------------------------------------------------------
    def _tick_build_legs(self, step) -> bool:
        if self._last_scan is None or self._last_scan not in self._pending:
            return True   # nothing to splice
        letter = self._last_scan
        spoke = step["spoke"]
        legs = self._pickplace_steps(letter, spoke)
        insert_at = self.step_idx + 1
        self.mission[insert_at:insert_at] = legs
        # Remove letter from pending; it'll either succeed or be re-checked next pass.
        if letter in self._pending:
            self._pending.remove(letter)
        self.get_logger().info(
            f"  BUILD_LEGS: spliced {len(legs)} steps for `{letter}` at spoke `{spoke}`. "
            f"pending={self._pending}")
        self._last_scan = None
        return True


def main():
    rclpy.init()
    node = RobosotMission()
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
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
