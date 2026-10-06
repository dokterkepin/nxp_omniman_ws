#!/usr/bin/env python3
"""
The arm's joint state for Cyclo: only the arm's joints, in the order the
policies were trained on.

    in   /joint_states        sensor_msgs/JointState   every joint of the robot
    out  /omniman/arm_state   sensor_msgs/JointState   state.arm.joint_names of
                                                       omniman_config.yaml, in that order

Cyclo's robot client takes a whole JointState message as the state of one
joint group, in message order. The AI Worker publishes one message per limb, so
that works for it. omniman's /joint_states carries every joint - the four wheels
too - in alphabetical order, so the policy was given the wheels' positions where
the shoulder and wrist angles belong ("state dim mismatch: got 11, policy
expects 7 - truncating to 7") and ran badly. The robot configs read the arm's
state from /omniman/arm_state instead, for inference and for recording.

Started by omniman_cyclo_bringup.launch.py.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import yaml

CONFIG = Path(get_package_share_directory('shared')) / 'robot_configs' / 'omniman_config.yaml'


def arm_joint_names():
    with open(CONFIG) as f:
        section = yaml.safe_load(f)['orchestrator']['ros__parameters']['omniman']
    return list(section['observation']['state']['arm']['joint_names'])


class ArmStateRelay(Node):

    def __init__(self):
        super().__init__('arm_state_relay')
        self.names = arm_joint_names()
        self.index = None            # positions of self.names in the incoming message
        self.last = ()               # the incoming name list self.index was made for
        self.warned = False
        self.pub = self.create_publisher(JointState, '/omniman/arm_state', QoSProfile(depth=10))
        self.create_subscription(JointState, '/joint_states', self.on_state, QoSProfile(depth=10))
        self.get_logger().info(f'/joint_states -> /omniman/arm_state: {", ".join(self.names)}')

    def on_state(self, msg):
        if self.index is None or self.last != tuple(msg.name):
            lookup = {name: i for i, name in enumerate(msg.name)}
            missing = [n for n in self.names if n not in lookup]
            if missing:
                if not self.warned:
                    self.get_logger().error(f'/joint_states lacks {missing} - nothing published')
                    self.warned = True
                return
            self.index = [lookup[n] for n in self.names]
            self.last = tuple(msg.name)
            self.warned = False
        out = JointState()
        out.header = msg.header
        out.name = self.names
        out.position = [msg.position[i] for i in self.index]
        if len(msg.velocity) == len(msg.name):
            out.velocity = [msg.velocity[i] for i in self.index]
        if len(msg.effort) == len(msg.name):
            out.effort = [msg.effort[i] for i in self.index]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ArmStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
