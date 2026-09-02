#!/usr/bin/env python3
"""
Replay ONLY the recorded base velocities of one episode onto /cmd_vel.

Answers "is the fault in the data/wiring, or in the policy?" -- if the wheels
move here, everything below the policy is fine.

Edit the constants below. Stop physical_ai_server inference (FINISH) first,
otherwise it publishes to /cmd_vel too and the two fight.
"""

import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# ---------------------------------------------------------------- edit these
EPISODE = 32
ROOT = '/home/dokterkepin/dataset/dokterkepin/omniman_nav2_drive_pick_place'
REPO_ID = 'dokterkepin/omniman_nav2_drive_pick_place'
SPEED = 1.0        # 0.5 = half speed
# ---------------------------------------------------------------------------

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'physical_ai_tools' / 'lerobot' / 'src'))
from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402


def main():
    ds = LeRobotDataset(REPO_ID, root=ROOT, episodes=[EPISODE])
    actions = ds.hf_dataset.select_columns('action')

    rclpy.init()
    node = Node('replay_base')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    node.get_logger().info(
        f'episode {EPISODE}: {ds.num_frames} frames @ {ds.fps}fps, starting in 3s')
    time.sleep(3.0)

    period = 1.0 / (ds.fps * SPEED)
    try:
        for i in range(ds.num_frames):
            t0 = time.perf_counter()
            a = actions[i]['action']
            msg = Twist()
            msg.linear.x = float(a[7])
            msg.linear.y = float(a[8])
            msg.angular.z = float(a[9])
            pub.publish(msg)

            if i % 60 == 0:
                node.get_logger().info(
                    f'  {i:5d}/{ds.num_frames}  '
                    f'[{a[7]:+.3f} {a[8]:+.3f} {a[9]:+.3f}]')

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted')
    finally:
        # Never leave the base rolling, on any exit path.
        pub.publish(Twist())
        time.sleep(0.1)
        pub.publish(Twist())
        node.get_logger().info('base stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
