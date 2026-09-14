#!/usr/bin/env python3
"""
Replay ONLY the recorded arm+gripper positions of one episode onto
/leader/joint_trajectory (the same topic the policy publishes at inference).

Edit the constants below.

SAFETY: the arm jumps straight to the episode's first pose -- put the follower
near its ready pose first and keep the e-stop reachable.
Requires teleop_bridges_launch.py running (it relays /leader/joint_trajectory
to /arm_controller/joint_trajectory); without it nothing moves.
Stop physical_ai_server inference (FINISH) first or both publish at once.
"""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------- edit these
EPISODE = 61
ROOT = '/home/dokterkepin/dataset/dokterkepin/omniman_nav2_drive_pick_place'
REPO_ID = 'dokterkepin/omniman_nav2_drive_pick_place'
SPEED = 1.0        # 0.5 = half speed
# ---------------------------------------------------------------------------

JOINTS = [
    'shoulder_yaw_joint',
    'upper_shoulder_pitch_joint',
    'arm_yaw_joint',
    'forearm_pitch_joint',
    'wrist_pitch_joint',
    'palm_yaw_joint',
    'gripper_prismatic_joint',
]

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'physical_ai_tools' / 'lerobot' / 'src'))
from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402


def main():
    ds = LeRobotDataset(REPO_ID, root=ROOT, episodes=[EPISODE])
    actions = ds.hf_dataset.select_columns('action')

    rclpy.init()
    node = Node('replay_arm')
    pub = node.create_publisher(JointTrajectory, '/leader/joint_trajectory', 10)

    first = actions[0]['action'][:7]
    node.get_logger().info(
        f'episode {EPISODE}: {ds.num_frames} frames @ {ds.fps}fps')
    node.get_logger().warn(
        'arm will JUMP to: ' + ' '.join(f'{v:+.3f}' for v in first))
    node.get_logger().info('starting in 3s -- Ctrl-C to abort')
    time.sleep(3.0)

    period = 1.0 / (ds.fps * SPEED)
    try:
        for i in range(ds.num_frames):
            t0 = time.perf_counter()
            a = actions[i]['action']
            msg = JointTrajectory(joint_names=JOINTS)
            # No time_from_start -- matches what physical_ai_server publishes,
            # which the controller treats as "go there now".
            msg.points = [JointTrajectoryPoint(
                positions=[float(v) for v in a[:7]])]
            pub.publish(msg)

            if i % 60 == 0:
                node.get_logger().info(
                    f'  {i:5d}/{ds.num_frames}  grip={a[6]:+.4f}')

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted -- arm holds its last commanded pose')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
