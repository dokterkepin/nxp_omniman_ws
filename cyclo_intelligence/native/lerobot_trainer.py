#!/usr/bin/env python3
"""
Cyclo's LeRobot training service, natively: /lerobot/train, /lerobot/stop
and training status - what the orchestrator's training manager calls when the
UI's Training page starts a run.

Cyclo 1.4.0 ships both halves - the service framework
(cyclo_brain/sdk/robot_client: RobotServiceServer) and the training logic
(cyclo_brain/policy/lerobot/training.py, around LeRobot's own lerobot_train)
- but no process that joins them. This is that process, nothing more; it runs
in the cyclo_lerobot conda env with the policy runtime (backends.py).
"""

import os

from robot_client.service_server import RobotServiceServer
import training   # cyclo_brain/policy/lerobot/training.py

server = RobotServiceServer(
    name=os.environ.get('POLICY_BACKEND', 'lerobot'),
    router_ip=os.environ.get('ZENOH_ROUTER_IP', '127.0.0.1'),
    router_port=int(os.environ.get('ZENOH_ROUTER_PORT', '7447')),
    domain_id=int(os.environ.get('ROS_DOMAIN_ID', '0')),
)


@server.on_train
def handle_train(request):
    training.run_training(server, request)


@server.on_stop
def handle_stop():
    training.cleanup_training()


if __name__ == '__main__':
    server.spin()
