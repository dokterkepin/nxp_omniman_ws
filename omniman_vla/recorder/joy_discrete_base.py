#!/usr/bin/env python3
"""Discrete-speed joystick driver for the mecanum base, LeKiwi style.

WHY THIS EXISTS
    teleop_twist_joy is proportional: stick deflection maps continuously to
    speed. LeKiwi instead drives its base from the keyboard, so every base
    command is either 0 or one of three fixed levels, and that is the data its
    policies are trained on (lerobot/robots/lekiwi/lekiwi_client.py).

    This node reproduces that on a joystick: a direction is pressed or it is
    not, and the speed comes from the currently selected level.

    Replaces teleop_twist_joy; do not run both, they both publish /cmd_vel.

SPEED LEVELS
    Defaults match LeKiwi exactly: 0.1 / 0.25 / 0.4 m/s and 30 / 60 / 90 deg/s.
    Published angular.z is in rad/s, because that is what /cmd_vel means here;
    the deg/s figures above are just how LeKiwi's table is written.

FINDING YOUR BUTTON NUMBERS
    Bluetooth and USB pads number things differently. Run with debug enabled
    and press one control at a time:

        ros2 run omniman_vla joy_discrete_base.py --ros-args -p debug:=true

    It logs the index of whatever axis or button changed. Put those numbers in
    config/joystick_discrete.yaml.

    D-pads appear as axes on some pads and as buttons on others, so both are
    supported. Set the ones you do not use to -1.

SAFETY
    Publishes zero if /joy goes quiet for joy_timeout seconds, and zero on
    shutdown, so a dropped Bluetooth link stops the base instead of leaving the
    last command latched.

TURNING IT OFF FOR INFERENCE
    physical_ai_server also publishes /cmd_vel during inference, and this node
    streams zeros whenever it runs, which drags every policy command back
    toward zero. Set teleop_enabled false and this node publishes nothing:

        ros2 param set /joy_discrete_base teleop_enabled false   # inference
        ros2 param set /joy_discrete_base teleop_enabled true    # teleop / recording

    Switching it off sends one stop first, so the base does not keep coasting
    on the last joystick command.
"""

import math
import signal
import time

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy

# An analog stick resting near zero must not count as "pressed"; anything past
# this counts as full deflection, which is what makes the output discrete even
# when the control is an analog axis.
AXIS_THRESHOLD = 0.5


class JoyDiscreteBase(Node):

    def __init__(self):
        super().__init__('joy_discrete_base')

        p = self.declare_parameter
        # false = publish nothing on /cmd_vel (use during inference).
        p('teleop_enabled', True)
        p('publish_rate', 30.0)
        p('joy_timeout', 0.5)
        p('debug', False)

        # Speed levels, slowest first. The two lists must be the same length.
        p('speed_xy', [0.1, 0.25, 0.4])            # m/s
        p('speed_theta_deg', [30.0, 60.0, 90.0])   # deg/s
        p('start_level', 0)

        # D-pad as axes (set to -1 if your pad reports the D-pad as buttons).
        p('axis_forward', 7)
        p('axis_strafe', 6)
        p('invert_forward', False)
        p('invert_strafe', False)

        # D-pad as buttons (set to -1 if you used the axes above).
        p('button_forward', -1)
        p('button_backward', -1)
        p('button_left', -1)
        p('button_right', -1)

        # Rotation and speed-level control.
        p('button_turn_left', 4)
        p('button_turn_right', 5)
        p('button_speed_up', 3)
        p('button_speed_down', 0)

        # Optional hold-to-drive button; -1 disables it.
        p('button_enable', -1)

        g = self.get_parameter
        self.speed_xy = list(g('speed_xy').value)
        self.speed_theta = [math.radians(v) for v in g('speed_theta_deg').value]
        if len(self.speed_xy) != len(self.speed_theta):
            raise ValueError('speed_xy and speed_theta_deg must be the same length')
        self.level = min(max(int(g('start_level').value), 0), len(self.speed_xy) - 1)

        self.teleop_enabled = bool(g('teleop_enabled').value)
        self.joy_timeout = float(g('joy_timeout').value)
        self.debug = bool(g('debug').value)
        self.add_on_set_parameters_callback(self._on_params)

        self.axis_forward = int(g('axis_forward').value)
        self.axis_strafe = int(g('axis_strafe').value)
        self.sign_forward = -1.0 if g('invert_forward').value else 1.0
        self.sign_strafe = -1.0 if g('invert_strafe').value else 1.0

        self.btn = {n: int(g(f'button_{n}').value) for n in (
            'forward', 'backward', 'left', 'right',
            'turn_left', 'turn_right', 'speed_up', 'speed_down', 'enable')}

        self.joy = None
        self.last_joy_time = None
        self.prev_buttons = []

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_timer(1.0 / float(g('publish_rate').value), self._tick)

        self.get_logger().info(
            f'discrete base teleop ready - levels {self.speed_xy} m/s / '
            f'{g("speed_theta_deg").value} deg/s, starting at level {self.level}')

    def _on_params(self, params):
        for prm in params:
            if prm.name == 'teleop_enabled':
                on = bool(prm.value)
                if self.teleop_enabled and not on:
                    self.stop()
                self.teleop_enabled = on
                self.get_logger().info(
                    'teleop ENABLED - publishing /cmd_vel' if on
                    else 'teleop DISABLED - /cmd_vel left to other publishers')
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------- input

    def _on_joy(self, msg: Joy):
        if self.debug:
            self._log_changes(msg)

        # Speed level changes on the press edge only, so holding the button
        # does not run through every level.
        for name, delta in (('speed_up', 1), ('speed_down', -1)):
            if self._pressed_edge(msg, self.btn[name]):
                new = min(max(self.level + delta, 0), len(self.speed_xy) - 1)
                if new != self.level:
                    self.level = new
                    self.get_logger().info(
                        f'speed level {self.level} '
                        f'({self.speed_xy[self.level]:.2f} m/s, '
                        f'{math.degrees(self.speed_theta[self.level]):.0f} deg/s)')

        self.prev_buttons = list(msg.buttons)
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def _log_changes(self, msg: Joy):
        for i, v in enumerate(msg.axes):
            if abs(v) > AXIS_THRESHOLD:
                self.get_logger().info(f'axis {i} = {v:+.2f}')
        for i, v in enumerate(msg.buttons):
            if v and (i >= len(self.prev_buttons) or not self.prev_buttons[i]):
                self.get_logger().info(f'button {i} pressed')

    def _pressed_edge(self, msg: Joy, idx: int) -> bool:
        if idx < 0 or idx >= len(msg.buttons):
            return False
        was = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return bool(msg.buttons[idx]) and not was

    def _held(self, name: str) -> bool:
        idx = self.btn[name]
        if idx < 0 or self.joy is None or idx >= len(self.joy.buttons):
            return False
        return bool(self.joy.buttons[idx])

    def _axis_sign(self, idx: int, sign: float) -> float:
        """+1, -1 or 0 - never a partial value, which is the point of this node."""
        if idx < 0 or self.joy is None or idx >= len(self.joy.axes):
            return 0.0
        v = self.joy.axes[idx] * sign
        if v > AXIS_THRESHOLD:
            return 1.0
        if v < -AXIS_THRESHOLD:
            return -1.0
        return 0.0

    # --------------------------------------------------------------- output

    def _tick(self):
        if not self.teleop_enabled:
            return

        twist = Twist()

        stale = (
            self.joy is None
            or (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
            > self.joy_timeout
        )
        enabled = self.btn['enable'] < 0 or self._held('enable')

        if not stale and enabled:
            xy = self.speed_xy[self.level]
            theta = self.speed_theta[self.level]

            fwd = self._axis_sign(self.axis_forward, self.sign_forward)
            if fwd == 0.0:
                fwd = float(self._held('forward')) - float(self._held('backward'))

            strafe = self._axis_sign(self.axis_strafe, self.sign_strafe)
            if strafe == 0.0:
                strafe = float(self._held('left')) - float(self._held('right'))

            turn = float(self._held('turn_left')) - float(self._held('turn_right'))

            twist.linear.x = fwd * xy
            twist.linear.y = strafe * xy
            twist.angular.z = turn * theta

        self.pub.publish(twist)

    def stop(self):
        """Send zeros a few times; one message can be lost on a closing link."""
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.01)


def main():
    # rclpy's own signal handlers invalidate the context BEFORE we get a chance
    # to publish, so the stop command would fail with "publisher's context is
    # invalid" and the controller would sit on the last velocity it was given
    # (mecanum reference_timeout is 10 s). Handle the signals ourselves so the
    # base is zeroed while the context is still alive.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JoyDiscreteBase()

    def _on_signal(_signum, _frame):
        # When disabled a policy owns /cmd_vel; don't interrupt it with zeros.
        if node.teleop_enabled:
            node.stop()
        raise SystemExit

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
