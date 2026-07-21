#!/usr/bin/env python3
#
# Gripper teleop bridge: leader trigger -> follower gripper.
#
# The arm is bridged by a plain topic relay (leader JTC -> follower JTC), but the
# gripper can't be, because omniman's gripper is a *GripperActionController*
# (action-based) on a *prismatic* joint measured in metres, while the leader
# "gripper" is a *revolute* trigger measured in radians with a different name.
#
# This node closes that gap: it reads the leader trigger angle from
# /leader/joint_states, linearly remaps it into the follower's finger-travel
# range, and sends it as a GripperCommand action goal to /gripper_controller.
#
# Every mapping parameter is read live each cycle, so the endpoints and the
# invert flag can be calibrated on real hardware with `ros2 param set` without
# restarting the node.

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand


class GripperTeleopBridge(Node):
    def __init__(self):
        super().__init__('gripper_teleop_bridge')

        # --- Parameters (all live-tunable via `ros2 param set`) ---
        # Which joint on the leader carries the trigger value.
        self.declare_parameter('leader_joint_name', 'gripper_prismatic_joint')
        # Leader trigger range (radians). This is the mapping window, NOT the URDF
        # limit — the follower clamps at its own max/min when the trigger goes
        # outside [leader_min, leader_max]. Tuned so a small squeeze covers the
        # full gripper range; the trigger can physically travel past leader_max.
        self.declare_parameter('leader_min', -0.4)
        self.declare_parameter('leader_max', 0.0)
        # Follower finger range (metres), from omniman left_finger_prismatic_joint.
        self.declare_parameter('follower_min', -0.010)
        self.declare_parameter('follower_max', 0.019)
        # Flip mapping direction (squeeze -> close vs squeeze -> open).
        self.declare_parameter('invert', False)
        # GripperCommand effort cap (URDF effort limit is 1).
        self.declare_parameter('max_effort', 1.0)
        # How often to (re)send a goal, and the smallest change worth sending.
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('min_change', 0.0005)  # metres
        # Action server for omniman's gripper_controller.
        self.declare_parameter('action_name', '/gripper_controller/gripper_cmd')

        leader_joint = self.get_parameter('leader_joint_name').value
        self._leader_joint = leader_joint

        self._leader_value = None      # latest raw trigger reading
        self._last_sent = None         # last target we actually commanded

        self._sub = self.create_subscription(
            JointState, '/leader/joint_states', self._joint_state_cb, 10)

        action_name = self.get_parameter('action_name').value
        self._action_client = ActionClient(self, GripperCommand, action_name)

        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"gripper_teleop_bridge: mapping leader '{leader_joint}' "
            f"-> action '{action_name}'. Waiting for action server...")
        self._server_ready = False

    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index(self._leader_joint)
        except ValueError:
            return  # leader joint not in this message
        self._leader_value = msg.position[idx]

    def _map_to_follower(self, leader_val: float) -> float:
        leader_min = self.get_parameter('leader_min').value
        leader_max = self.get_parameter('leader_max').value
        follower_min = self.get_parameter('follower_min').value
        follower_max = self.get_parameter('follower_max').value
        invert = self.get_parameter('invert').value

        span = leader_max - leader_min
        if abs(span) < 1e-9:
            return follower_min
        frac = (leader_val - leader_min) / span
        frac = max(0.0, min(1.0, frac))   # clamp to [0, 1]
        if invert:
            frac = 1.0 - frac
        return follower_min + frac * (follower_max - follower_min)

    def _on_timer(self):
        if self._leader_value is None:
            return
        if not self._action_client.server_is_ready():
            if self._server_ready:
                self.get_logger().warn('gripper action server went away.')
                self._server_ready = False
            return
        if not self._server_ready:
            self.get_logger().info('gripper action server connected.')
            self._server_ready = True

        target = self._map_to_follower(self._leader_value)
        min_change = self.get_parameter('min_change').value
        if self._last_sent is not None and abs(target - self._last_sent) < min_change:
            return  # not enough change to bother re-commanding

        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = self.get_parameter('max_effort').value
        # Fire-and-forget: GripperActionController preempts the prior goal.
        self._action_client.send_goal_async(goal)
        self._last_sent = target


def main():
    rclpy.init()
    node = GripperTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
