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
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import cv2
import mediapipe as mp

from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from hand_utils import quaternion_from_euler_numpy, get_hand_pose_from_landmarks, get_finger_states

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Which hand controls the arm. MediaPipe handedness labels are mirror-correct
# after we flip the frame, so "Right" == the user's right hand.
CONTROL_HAND = "right"


class OmnimanHandTeleop(Node):
    def __init__(self):
        super().__init__("hand_pose_publisher_node")

        # ---- Parameters --------------------------------------------------
        self.camera_device = self.declare_parameter("camera_device", 2).value          # /dev/video2 (C920)
        self.planning_frame = self.declare_parameter("planning_frame", "base_link").value
        self.gripper_action = self.declare_parameter(
            "gripper_action", "/gripper_controller/gripper_cmd").value

        # Robot workspace box (planning frame, metres). Defaults for omniman.
        self.robot_x_min = self.declare_parameter("robot_x_min", 0.15).value   # forward, near
        self.robot_x_max = self.declare_parameter("robot_x_max", 0.40).value   # forward, far
        self.robot_y_min = self.declare_parameter("robot_y_min", -0.20).value  # lateral
        self.robot_y_max = self.declare_parameter("robot_y_max", 0.20).value
        self.robot_z_min = self.declare_parameter("robot_z_min", 0.25).value   # height
        self.robot_z_max = self.declare_parameter("robot_z_max", 0.50).value

        # Hand-depth (MediaPipe relative z of middle-finger MCP) -> robot X.
        self.depth_near = self.declare_parameter("depth_near", -0.15).value
        self.depth_far = self.declare_parameter("depth_far", 0.01).value

        # Gripper limits (omniman left_finger_prismatic_joint: -0.010..0.019)
        self.gripper_open = self.declare_parameter("gripper_open_position", 0.019).value
        self.gripper_close = self.declare_parameter("gripper_close_position", -0.010).value
        self.gripper_effort = self.declare_parameter("gripper_max_effort", 1.0).value
        self.open_finger_count = self.declare_parameter("open_finger_count", 2).value

        self.show_window = self.declare_parameter("show_window", True).value

        # ---- ROS interfaces ---------------------------------------------
        self.pose_pub = self.create_publisher(PoseStamped, "/hand_target_pose", 10)
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

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info(
            f"Omniman hand teleop started. Camera=/dev/video{self.camera_device}, "
            f"controlling with the {CONTROL_HAND.upper()} hand. "
            f"Raise {self.open_finger_count}+ fingers = OPEN gripper.")

    # ---------------------------------------------------------------------
    def publish_arm_pose(self, hand_landmarks, w, h):
        """Map the right hand to a target EE pose and publish it."""
        cx, cy, _roll = get_hand_pose_from_landmarks(hand_landmarks.landmark, w, h)

        # Depth from middle-finger MCP z (wrist z is always ~0 in MediaPipe).
        z_val = hand_landmarks.landmark[9].z
        z_clamped = max(min(z_val, self.depth_far), self.depth_near)
        z_norm = (z_clamped - self.depth_near) / (self.depth_far - self.depth_near)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.planning_frame

        # depth -> X (forward), horizontal -> Y, vertical (inverted) -> Z
        pose.pose.position.x = self.robot_x_min + z_norm * (self.robot_x_max - self.robot_x_min)
        pose.pose.position.y = self.robot_y_min + cx * (self.robot_y_max - self.robot_y_min)
        pose.pose.position.z = self.robot_z_min + (1.0 - cy) * (self.robot_z_max - self.robot_z_min)

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
        if results.multi_hand_landmarks and results.multi_handedness:
            for landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label.lower()
                mp_drawing.draw_landmarks(frame, landmarks, mp_hands.HAND_CONNECTIONS)
                if label == CONTROL_HAND:
                    right_hand = landmarks

        if right_hand is not None:
            pose = self.publish_arm_pose(right_hand, w, h)
            fingers = get_finger_states(right_hand.landmark)
            fingers_up = sum(fingers)
            want_open = fingers_up >= self.open_finger_count
            self.command_gripper(want_open)

            if self.show_window:
                cv2.putText(frame, "RIGHT HAND (controlling arm)", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(frame, f"X:{pose.pose.position.x:.2f} Y:{pose.pose.position.y:.2f} "
                                   f"Z:{pose.pose.position.z:.2f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(frame, f"Fingers:{fingers_up} | {'OPEN' if want_open else 'CLOSE'}",
                            (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Pose [{pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, "
                    f"{pose.pose.position.z:.2f}]  fingers={fingers_up}  "
                    f"gripper={'OPEN' if want_open else 'CLOSE'}")
        elif self.show_window:
            cv2.putText(frame, "Show your RIGHT hand", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if self.show_window:
            cv2.putText(frame, f"{self.open_finger_count}+ fingers = OPEN, fewer = CLOSE | 'q' quits",
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Omniman Hand Teleop", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()

    def destroy_node(self):
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