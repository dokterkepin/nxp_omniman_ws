#!/usr/bin/env python3
"""
Is the gripper holding something? Read from the gripper joint alone.

    in   /joint_states                  sensor_msgs/JointState
    out  /gripper/holding               std_msgs/Bool     latched, on change
         ~/state                        std_msgs/String   latched, on change:
              holding | empty - closed on nothing | empty - open  (+ readings)

The policy closes the gripper whether or not the cup is there, so "closed"
says nothing. How far it closes, and how hard it pushes, do:

    closed on the cup   fingers stop against it: position stays above
                        closed_empty_position (-0.002), and the servo pushes
                        at its current limit: |effort| ~20-200
    closed on nothing   fingers travel on toward fully closed (-0.013):
                        position below -0.002, and |effort| near 0
    open                position ~0.019, |effort| near 0

    holding = position > closed_empty_position  and  |effort| > holding_min_effort

both true for stable_s - a single sample while the fingers are still moving
or a current spike does not flip it. This is the check grippers do
themselves (ROS 2 parallel_gripper_action_controller's "stalled" result,
Robotiq's "object detected") and QT-Opt's grasp-success label (gripper not
fully closed), done on /joint_states because the gripper is commanded by the
policy through arm_controller, where no gripper controller can run.
The gripper is a Dynamixel in current-based position mode (operating_mode 5,
goal_current 200), which is what makes |effort| meaningful.

Settings: config/mission.yaml, section grasp_monitor - read from the file
directly (no ROS parameters); saved edits apply within a second.

Run:
  ros2 run omniman_vla grasp_monitor.py
  ros2 topic echo /grasp_monitor/state
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

SETTINGS = ['joint_states_topic', 'joint', 'closed_empty_position', 'holding_min_effort',
            'stable_s']

CONFIG_FILE = os.path.join(get_package_share_directory('omniman_vla'), 'config',
                           'mission.yaml')


class Config:
    """This node's section of config/mission.yaml - read straight from the
    file, not through ROS parameters. Re-read when the file changes (checked
    at most once a second), so saved edits apply without a restart; an edit
    that breaks the file is logged and the previous values are kept."""

    def __init__(self, section, missing, logger):
        # missing(values) -> names of the settings the section lacks.
        self.section, self.missing, self.logger = section, missing, logger
        self.checked = 0.0
        self.load()

    def load(self):
        mtime = os.stat(CONFIG_FILE).st_mtime
        with open(CONFIG_FILE) as f:
            values = (yaml.safe_load(f) or {}).get(self.section) or {}
        missing = self.missing(values)
        if missing:
            raise RuntimeError(f'{CONFIG_FILE} [{self.section}] is missing: '
                               f'{", ".join(missing)}')
        self.values, self.mtime = values, mtime

    def __getitem__(self, key):
        now = time.monotonic()
        if now - self.checked > 1.0:
            self.checked = now
            try:
                if os.stat(CONFIG_FILE).st_mtime != self.mtime:
                    self.load()
                    self.logger.info(f'reloaded {CONFIG_FILE}')
            except (OSError, yaml.YAMLError, RuntimeError, AttributeError) as e:
                self.logger.error(f'bad edit, keeping previous values: {e}')
                self.mtime = os.stat(CONFIG_FILE).st_mtime
        return self.values[key]


class GraspMonitor(Node):

    def __init__(self):
        super().__init__('grasp_monitor')
        # Every value comes from config/mission.yaml [grasp_monitor].
        self.cfg = Config('grasp_monitor', lambda v: [k for k in SETTINGS if k not in v],
                          self.get_logger())

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.holding_pub = self.create_publisher(Bool, '/gripper/holding', latched)
        self.state_pub = self.create_publisher(String, '~/state', latched)

        self.state = None        # published state; None until first decided
        self.pending = None      # reading waiting out stable_s
        self.pending_since = 0.0
        self.warned = False

        self.create_subscription(JointState, self.cfg['joint_states_topic'],
                                 self.on_joint_states, 10)
        self.get_logger().info(f'ready - watching {self.cfg["joint"]}')

    def on_joint_states(self, msg):
        joint = self.cfg['joint']
        if joint not in msg.name:
            return
        i = msg.name.index(joint)
        if i >= len(msg.effort):
            if not self.warned:
                self.get_logger().error(
                    f'no effort for {joint} in {self.cfg["joint_states_topic"]} - '
                    'cannot tell holding from open; reporting not holding')
                self.warned = True
            self.decide('empty - no effort reading', msg.position[i], float('nan'))
            return
        pos, eff = msg.position[i], msg.effort[i]
        if pos > self.cfg['closed_empty_position'] and abs(eff) > self.cfg['holding_min_effort']:
            state = 'holding'
        elif pos <= self.cfg['closed_empty_position']:
            state = 'empty - closed on nothing'
        else:
            state = 'empty - open'
        self.decide(state, pos, eff)

    def decide(self, state, pos, eff):
        """Publish a new state once the reading has held for stable_s."""
        now = time.monotonic()
        if state != self.pending:
            self.pending, self.pending_since = state, now
        if state == self.state or now - self.pending_since < self.cfg['stable_s']:
            return
        self.state = state
        self.holding_pub.publish(Bool(data=state == 'holding'))
        text = f'{state} (position {pos:+.4f}, effort {eff:+.0f})'
        self.state_pub.publish(String(data=text))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = GraspMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
