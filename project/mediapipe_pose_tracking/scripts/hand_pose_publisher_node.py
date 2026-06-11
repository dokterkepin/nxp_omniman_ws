#!/usr/bin/env python3
"""
MediaPipe hand-tracking teleop for the single omniman arm.

Tracks the RIGHT hand with a webcam and:
  * publishes a target end-effector pose -> /hand_target_pose
    (geometry_msgs/PoseStamped, in the servo planning frame, default base_link)
  * opens/closes the gripper via the GripperCommand action
    (/gripper_controller/gripper_cmd) based on how many fingers are raised.

The omniman_pose_tracking_node (C++) consumes /hand_target_pose and servos the
arm with MoveIt Servo.  Orientation from the hand is ignored by default on the
C++ side (position-only), because the 6-DOF arm has no null space.

Adapted from the dual-arm demo for a single arm + a GripperActionController.

All workspace / camera / gripper values are ROS parameters so you can tune them
live (ros2 param set) without rebuilding.  Defaults are sized for the compact
omniman arm (~0.45 m reach); tune them to your robot.

Dataset recording (for the LSTM trajectory-prediction project): press 'r' in the
OpenCV window to start/stop recording. Each frame with a tracked right hand is
appended to a CSV in `record_dir` (default: this package's dataset/) with both the RAW
mapped position (rx,ry,rz -- pre-smoothing) and the EMA output (fx,fy,fz). The
robot does NOT need to be running. Train with
notebooks/train_lstm_hand_trajectory.ipynb.

LSTM trajectory predictor (the model trained by that notebook): when `use_lstm`
is true (default) and the exported files exist in `model_dir`, the published
target is the LSTM's ~100 ms-ahead predicted position instead of the lagging
EMA. Press 'm' to toggle LSTM/EMA live (A/B comparison on the robot). The EMA
remains the fallback while the input window fills (~0.5 s after the hand
appears) or if the model cannot be loaded.
"""
# import math  # needed when workspace_angle zone rotation is re-enabled
import csv
import json
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import cv2
import mediapipe as mp

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from control_msgs.action import GripperCommand
from project.mediapipe_pose_tracking.scripts.hand_utils import quaternion_from_euler_numpy, get_hand_pose_from_landmarks, get_finger_states

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Which hand controls the arm. MediaPipe handedness labels are mirror-correct
# after we flip the frame, so "Right" == the user's right hand.
CONTROL_HAND = "right"


class OmnimanHandTeleop(Node):
    def __init__(self):
        super().__init__("hand_pose_publisher_node")

        # ---- Parameters --------------------------------------------------
        self.camera_device = int(self.declare_parameter("camera_device", 2).value)      
        self.planning_frame = self.declare_parameter("planning_frame", "base_link").value
        self.gripper_action = self.declare_parameter(
            "gripper_action", "/gripper_controller/gripper_cmd").value

        # Robot workspace box (planning frame, metres). Defaults for omniman.
        self.robot_x_min = self.declare_parameter("robot_x_min", 0.05).value   # forward, near (low allows side reach)
        self.robot_x_max = self.declare_parameter("robot_x_max", 0.40).value   # forward, far
        self.robot_y_min = self.declare_parameter("robot_y_min", -0.40).value  # lateral (wider for left/right reach)
        self.robot_y_max = self.declare_parameter("robot_y_max", 0.40).value
        self.robot_z_min = self.declare_parameter("robot_z_min", 0.10).value   # height (lowest reach)
        self.robot_z_max = self.declare_parameter("robot_z_max", 0.50).value

        # Forward reach (robot X) from hand depth. Two methods:
        #  - hand-size (default): apparent palm length in the image. Robust,
        #    monotonic: hand close to camera = bigger = reach forward.
        #  - mediapipe-z (legacy): landmark[9].z. Noisy/unreliable on a mono cam.
        self.use_hand_size_depth = self.declare_parameter("use_hand_size_depth", True).value
        # Palm length (wrist->middle-MCP, normalised image coords) calibration.
        # Hold your hand at the near/far extremes and read the on-screen "size=" to tune.
        self.hand_size_min = self.declare_parameter("hand_size_min", 0.05).value  # hand far  -> X min
        # hand close-> X max. Keep this comfortably BELOW a screen-filling palm so
        # full forward reach is hit while the hand is still well inside the frame
        # (room left for X/Y). Read the on-screen "size=" HUD to recalibrate.
        self.hand_size_max = self.declare_parameter("hand_size_max", 0.24).value
        # Legacy MediaPipe-z mapping (only used if use_hand_size_depth=False).
        self.depth_near = self.declare_parameter("depth_near", -0.15).value
        self.depth_far = self.declare_parameter("depth_far", 0.01).value
        # Exponential smoothing on the output pose (0..1, lower = smoother/laggier).
        self.pose_smoothing = self.declare_parameter("pose_smoothing", 0.3).value

        # Gripper limits (omniman left_finger_prismatic_joint: -0.010..0.019)
        self.gripper_open = self.declare_parameter("gripper_open_position", 0.019).value
        self.gripper_close = self.declare_parameter("gripper_close_position", -0.010).value
        self.gripper_effort = self.declare_parameter("gripper_max_effort", 1.0).value
        self.open_finger_count = self.declare_parameter("open_finger_count", 2).value

        # Left-hand "reset" gesture: an open LEFT palm publishes an SRDF named
        # pose (default "ready") to /arm_named_pose. The C++ node pauses servo
        # tracking and sends the arm there -- a singularity escape + orientation
        # reset. Needs the robot launched with use_trajectory:=true (JTC).
        self.reset_pose_name = self.declare_parameter("reset_pose_name", "ready").value
        self.reset_finger_count = self.declare_parameter("reset_finger_count", 4).value

        self.show_window = self.declare_parameter("show_window", True).value

        # Trajectory dataset recording ('r' key). CSVs land here; the LSTM
        # training notebook reads the same directory.
        self.record_dir = os.path.expanduser(
            self.declare_parameter(
                "record_dir",
                "~/workspaces/nxp_omniman_ws/src/mediapipe_pose_tracking/dataset",
            ).value)

        # LSTM trajectory predictor (exported by the training notebook).
        # 'm' key toggles LSTM/EMA live for A/B comparison.
        self.use_lstm = self.declare_parameter("use_lstm", True).value
        self.model_dir = os.path.expanduser(
            self.declare_parameter(
                "model_dir",
                "~/workspaces/nxp_omniman_ws/src/mediapipe_pose_tracking/models",
            ).value)

        # ---- ROS interfaces ---------------------------------------------
        self.pose_pub = self.create_publisher(PoseStamped, "/hand_target_pose", 10)
        self.named_pose_pub = self.create_publisher(String, "/arm_named_pose", 10)
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action)

        # ---- Camera + MediaPipe -----------------------------------------
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open camera device {self.camera_device}. "
                f"Set the 'camera_device' param (your C920 is /dev/video2 -> 2).")
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,            # detect both, but only act on the right hand
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )

        self.frame_count = 0
        self._gripper_open_state = None   # None until first command
        self._filt = None                 # EMA-smoothed [x, y, z]
        self._hand_size = 0.0             # last apparent palm length (for HUD)
        self._reset_sent = False          # latch: left-hand reset fires once per gesture

        # Recording state. A "segment" is an unbroken run of tracked frames;
        # it increments whenever the hand is lost so the training notebook
        # never builds an LSTM window across a tracking gap.
        self._rec_file = None
        self._rec_writer = None
        self._rec_count = 0
        self._rec_segment = 0
        self._rec_hand_lost = False

        self._pred_src = "EMA"            # what the last published target came from (HUD)
        self._load_lstm()

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info(
            f"Omniman hand teleop started. Camera=/dev/video{self.camera_device}, "
            f"controlling with the {CONTROL_HAND.upper()} hand. "
            f"Raise {self.open_finger_count}+ fingers = OPEN gripper.")

    # ---------------------------------------------------------------------
    def toggle_recording(self):
        """'r' key: start/stop appending hand positions to a timestamped CSV."""
        if self._rec_file is None:
            os.makedirs(self.record_dir, exist_ok=True)
            path = os.path.join(self.record_dir,
                                time.strftime("hand_traj_%Y%m%d_%H%M%S.csv"))
            self._rec_file = open(path, "w", newline="")
            self._rec_writer = csv.writer(self._rec_file)
            self._rec_writer.writerow(["t", "seg", "rx", "ry", "rz", "fx", "fy", "fz"])
            self._rec_count = 0
            self._rec_segment = 0
            self._rec_hand_lost = False
            self.get_logger().info(f"RECORDING hand trajectory -> {path}")
        else:
            path = self._rec_file.name
            self._rec_file.close()
            self._rec_file = None
            self._rec_writer = None
            self.get_logger().info(
                f"Recording stopped: {self._rec_count} samples, "
                f"{self._rec_segment + 1} segment(s) -> {path}")

    def record_sample(self, rx, ry, rz, fx, fy, fz):
        """Append one frame: raw mapped position + the node's EMA output."""
        if self._rec_writer is None:
            return
        if self._rec_hand_lost:
            self._rec_segment += 1
            self._rec_hand_lost = False
        self._rec_writer.writerow(
            [f"{time.monotonic():.4f}", self._rec_segment,
             f"{rx:.5f}", f"{ry:.5f}", f"{rz:.5f}",
             f"{fx:.5f}", f"{fy:.5f}", f"{fz:.5f}"])
        self._rec_count += 1

    # ---------------------------------------------------------------------
    def _load_lstm(self):
        """Load the TorchScript trajectory predictor exported by the notebook.
        On any failure the node simply keeps using the EMA."""
        self.lstm = None
        self.lstm_active = False
        if not self.use_lstm:
            return
        try:
            import torch
            with open(os.path.join(self.model_dir, "lstm_hand_traj.json")) as f:
                meta = json.load(f)
            torch.set_num_threads(1)   # tiny model; don't fight the camera loop
            self.lstm = torch.jit.load(
                os.path.join(self.model_dir, "lstm_hand_traj.torchscript.pt"),
                map_location="cpu")
            self.lstm.eval()
            self._torch = torch
            # model i/o is z-normalized displacement relative to the window's last frame
            self.lstm_mean = np.array(meta["input_mean"], dtype=np.float32)
            self.lstm_std = np.array(meta["input_std"], dtype=np.float32)
            self._raw_buf = deque(maxlen=int(meta["seq_len"]))
            with torch.no_grad():   # JIT warm-up: the first few calls are slow
                for _ in range(5):
                    self.lstm(torch.zeros(1, int(meta["seq_len"]), 3))
            self.lstm_active = True
            self.get_logger().info(
                f"LSTM predictor loaded: {meta['seq_len']}-frame window, "
                f"+{meta.get('lead_ms', '?')} ms lead, test MAE "
                f"{meta.get('test_mae_mm', '?')} mm. Press 'm' to toggle LSTM/EMA.")
        except Exception as e:
            self.lstm = None
            self.get_logger().warn(f"LSTM predictor unavailable ({e}); using EMA smoothing.")

    def lstm_predict(self, rx, ry, rz):
        """Push the raw position into the window and return the predicted
        position ~100 ms ahead, or None while the window is still filling."""
        self._raw_buf.append((rx, ry, rz))
        if len(self._raw_buf) < self._raw_buf.maxlen:
            return None
        win = np.asarray(self._raw_buf, dtype=np.float32)
        x = (win - win[-1] - self.lstm_mean) / self.lstm_std
        with self._torch.no_grad():
            out = self.lstm(self._torch.from_numpy(x).unsqueeze(0))[0].numpy()
        return win[-1] + out * self.lstm_std + self.lstm_mean

    # ---------------------------------------------------------------------
    def publish_arm_pose(self, hand_landmarks, w, h):
        """Map the right hand to a target EE pose and publish it."""
        lm = hand_landmarks.landmark
        cx, cy, _roll = get_hand_pose_from_landmarks(lm, w, h)

        # --- Forward reach (robot X) -------------------------------------
        if self.use_hand_size_depth:
            # Apparent palm length: wrist(0) -> middle-finger MCP(9), normalised
            # image coords. Stable, monotonic depth cue (closer hand = bigger).
            wrist, mid_mcp = lm[0], lm[9]
            self._hand_size = ((mid_mcp.x - wrist.x) ** 2 + (mid_mcp.y - wrist.y) ** 2) ** 0.5
            s = max(min(self._hand_size, self.hand_size_max), self.hand_size_min)
            x_norm = (s - self.hand_size_min) / (self.hand_size_max - self.hand_size_min)
        else:
            z_val = lm[9].z   # legacy: noisy MediaPipe per-landmark z
            z_clamped = max(min(z_val, self.depth_far), self.depth_near)
            x_norm = (z_clamped - self.depth_near) / (self.depth_far - self.depth_near)

        rx = self.robot_x_min + x_norm * (self.robot_x_max - self.robot_x_min)
        ry = self.robot_y_min + cx * (self.robot_y_max - self.robot_y_min)
        rz = self.robot_z_min + (1.0 - cy) * (self.robot_z_max - self.robot_z_min)

        # --- Exponential smoothing (tames jitter, esp. on depth) ---------
        a = self.pose_smoothing
        if self._filt is None:
            self._filt = [rx, ry, rz]
        else:
            self._filt[0] += a * (rx - self._filt[0])
            self._filt[1] += a * (ry - self._filt[1])
            self._filt[2] += a * (rz - self._filt[2])
        fx, fy, fz = self._filt

        self.record_sample(rx, ry, rz, fx, fy, fz)

        # --- Published target: LSTM prediction (~100 ms lead) or the EMA --
        tx, ty, tz = fx, fy, fz
        self._pred_src = "EMA"
        if self.lstm is not None:
            try:
                p = self.lstm_predict(rx, ry, rz)
            except Exception as e:
                self.get_logger().error(f"LSTM inference failed ({e}); reverting to EMA.")
                self.lstm = None
                p = None
            if p is not None and self.lstm_active:
                # never command outside the workspace box
                tx = min(max(float(p[0]), self.robot_x_min), self.robot_x_max)
                ty = min(max(float(p[1]), self.robot_y_min), self.robot_y_max)
                tz = min(max(float(p[2]), self.robot_z_min), self.robot_z_max)
                self._pred_src = "LSTM"

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = tx
        pose.pose.position.y = ty
        pose.pose.position.z = tz

        # Orientation is ignored by the C++ node in position-only mode; send
        # identity so the message is well-formed.
        q = quaternion_from_euler_numpy(0.0, 0.0, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)
        return pose

    def command_gripper(self, want_open):
        """Send a GripperCommand goal only when the open/close state changes."""
        if self._gripper_open_state == want_open:
            return
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                f"Gripper action server {self.gripper_action} not available yet.", once=True)
            return
        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_open if want_open else self.gripper_close
        goal.command.max_effort = self.gripper_effort
        self.gripper_client.send_goal_async(goal)
        self._gripper_open_state = want_open
        self.get_logger().info(f"Gripper -> {'OPEN' if want_open else 'CLOSE'} "
                               f"({goal.command.position:.3f} m)")

    def command_named_pose(self, left_hand):
        """Open LEFT palm publishes the reset/named pose once per gesture."""
        if left_hand is None:
            self._reset_sent = False
            return None
        lfingers = sum(get_finger_states(left_hand.landmark))
        trigger = lfingers >= self.reset_finger_count
        if trigger and not self._reset_sent:
            self.named_pose_pub.publish(String(data=self.reset_pose_name))
            self._reset_sent = True
            self.get_logger().info(f"LEFT hand reset -> named pose '{self.reset_pose_name}'")
        elif not trigger:
            self._reset_sent = False   # re-arm once the palm closes again
        return lfingers

    # ---------------------------------------------------------------------
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)   # mirror for natural interaction
        h, w, _ = frame.shape
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        self.frame_count += 1

        right_hand = None
        left_hand = None
        if results.multi_hand_landmarks and results.multi_handedness:
            for landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label.lower()
                mp_drawing.draw_landmarks(frame, landmarks, mp_hands.HAND_CONNECTIONS)
                if label == CONTROL_HAND:
                    right_hand = landmarks
                else:
                    left_hand = landmarks

        if right_hand is not None:
            pose = self.publish_arm_pose(right_hand, w, h)
            fingers = get_finger_states(right_hand.landmark)
            fingers_up = sum(fingers)
            want_open = fingers_up >= self.open_finger_count
            self.command_gripper(want_open)

            if self.show_window:
                cv2.putText(frame, f"RIGHT HAND (controlling arm) [{self._pred_src}]",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(frame, f"X:{pose.pose.position.x:.2f} Y:{pose.pose.position.y:.2f} "
                                   f"Z:{pose.pose.position.z:.2f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(frame, f"Fingers:{fingers_up} | {'OPEN' if want_open else 'CLOSE'}",
                            (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                if self.use_hand_size_depth:
                    cv2.putText(frame, f"size:{self._hand_size:.3f} "
                                       f"(min {self.hand_size_min:.2f}/max {self.hand_size_max:.2f})",
                                (10, 101), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Pose [{pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, "
                    f"{pose.pose.position.z:.2f}]  fingers={fingers_up}  "
                    f"gripper={'OPEN' if want_open else 'CLOSE'}")
        else:
            self._rec_hand_lost = True   # break the recording segment at gaps
            if self.lstm is not None:
                self._raw_buf.clear()    # never predict across a tracking gap
            if self.show_window:
                cv2.putText(frame, "Show your RIGHT hand", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # LEFT hand: open palm resets the arm to the "ready" SRDF pose.
        lfingers = self.command_named_pose(left_hand)
        if self.show_window and left_hand is not None:
            cv2.putText(frame, f"LEFT:{lfingers} (open palm = RESET '{self.reset_pose_name}')",
                        (10, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        if self.show_window:
            cv2.putText(frame, f"{self.open_finger_count}+ fingers = OPEN, fewer = CLOSE | "
                               f"'m' LSTM/EMA | 'r' rec | 'q' quits",
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            if self._rec_file is not None:
                cv2.circle(frame, (w - 160, 25), 8, (0, 0, 255), -1)
                cv2.putText(frame, f"REC {self._rec_count}", (w - 145, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("Omniman Hand Teleop", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
            elif key == ord("r"):
                self.toggle_recording()
            elif key == ord("m") and self.lstm is not None:
                self.lstm_active = not self.lstm_active
                self.get_logger().info(
                    f"Predictor -> {'LSTM' if self.lstm_active else 'EMA'}")

    def destroy_node(self):
        if self._rec_file is not None:
            self.toggle_recording()   # flush + close the CSV
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmnimanHandTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()