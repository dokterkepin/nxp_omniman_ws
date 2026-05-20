#!/usr/bin/env python3
"""Joystick → gripper bridge.

LEFT_STICK_CLICK  (button 9)  opens the gripper.
RIGHT_STICK_CLICK (button 10) closes the gripper.
While held, the target position steps by STEP_M each joy frame
(rate-limited to RATE_HZ), then a GripperCommand action goal is sent.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from control_msgs.action import GripperCommand


LEFT_STICK_CLICK = 9
RIGHT_STICK_CLICK = 10

OPEN_LIMIT = 0.019
CLOSE_LIMIT = -0.010
STEP_M = 0.001
MAX_EFFORT = 0.0
RATE_HZ = 20.0


class JoyToGripper(Node):
    def __init__(self):
        super().__init__("joy_to_gripper")
        self._target = OPEN_LIMIT
        self._min_period = 1.0 / RATE_HZ
        self._last_send = self.get_clock().now()

        self._client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.get_logger().info(
            f"joy_to_gripper ready — open=button[{LEFT_STICK_CLICK}], "
            f"close=button[{RIGHT_STICK_CLICK}], step={STEP_M*1000:.1f} mm"
        )

    def _on_joy(self, msg: Joy):
        if len(msg.buttons) <= max(LEFT_STICK_CLICK, RIGHT_STICK_CLICK):
            return
        open_pressed = bool(msg.buttons[LEFT_STICK_CLICK])
        close_pressed = bool(msg.buttons[RIGHT_STICK_CLICK])
        if not (open_pressed ^ close_pressed):
            return

        now = self.get_clock().now()
        if (now - self._last_send).nanoseconds * 1e-9 < self._min_period:
            return

        if open_pressed:
            self._target += STEP_M
        else:
            self._target -= STEP_M
        self._target = max(CLOSE_LIMIT, min(OPEN_LIMIT, self._target))

        if not self._client.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = self._target
        goal.command.max_effort = MAX_EFFORT
        self._client.send_goal_async(goal)
        self._last_send = now


def main():
    rclpy.init()
    node = JoyToGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
