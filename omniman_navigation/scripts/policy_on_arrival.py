#!/usr/bin/env python3
"""
Start an arm policy as soon as a Nav2 goal is reached - no fixed delays.

    Nav2 goal SUCCEEDED -> wait until the base is still -> START_INFERENCE
    new Nav2 goal       -> FINISH (the arm must not work while the base drives)

Any goal counts, e.g. Foxglove's goal pose tool or RViz's 2D Goal Pose.
"Still" means wheel odometry under STILL_LINEAR / STILL_ANGULAR for
still_time_s. This replaces the fixed settle_s sleep in pick_place_shuttle.py.

The policy runs until you call ~/stop, stop it from the physical_ai_manager
UI, or send a new goal. Only a policy this node started is ever finished by
it: FINISH also ends a recording, so a stray one would cut short a recording
started from the UI.

Prereqs:
  - nav2_launch.py
  - physical_ai_server_bringup.launch.py (serves /task/command)

Run:
  ros2 run omniman_navigation policy_on_arrival.py --ros-args \
      -p policy_path:=/path/to/pretrained_model -p instruction:="pick the object"

Stop:
  ros2 service call /policy_on_arrival/stop std_srvs/srv/Trigger
"""

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from physical_ai_interfaces.srv import SendCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

STILL_LINEAR = 0.01     # m/s
STILL_ANGULAR = 0.02    # rad/s
SETTLE_TIMEOUT_S = 5.0  # give up if the base has not stopped by then

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)


class PolicyOnArrival(Node):

    def __init__(self):
        super().__init__('policy_on_arrival')
        self.declare_parameter(
            'policy_path',
            '/home/dokterkepin/output/omniman_nav2_drive_pick_place/'
            'checkpoints/last/pretrained_model')
        self.declare_parameter('instruction', 'pick the object')
        self.declare_parameter('fps', 30)
        self.declare_parameter('still_time_s', 0.5)

        self.client = self.create_client(SendCommand, '/task/command')

        # idle -> settling (goal reached) -> starting (START sent) -> running
        self.state = 'idle'
        self.goal_status = {}       # goal uuid -> last status seen
        self.settle_started = None
        self.still_since = None     # None while the base moves
        self.last_odom = None

        # Action status topics are reliable + transient_local.
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_nav_status, status_qos)
        self.create_subscription(
            Odometry, '/mecanum_drive_controller/odometry', self.on_odom, 10)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.05, self.tick)

        self.get_logger().info('ready - waiting for a Nav2 goal')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        t = msg.twist.twist
        still = (abs(t.linear.x) < STILL_LINEAR
                 and abs(t.linear.y) < STILL_LINEAR
                 and abs(t.angular.z) < STILL_ANGULAR)
        self.last_odom = self.now()
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = self.last_odom

    def on_nav_status(self, msg):
        previous = self.goal_status
        # The list holds every goal the server still remembers, so rebuilding
        # it from each message also forgets expired goals.
        self.goal_status = {
            bytes(s.goal_info.goal_id.uuid): s.status for s in msg.status_list}

        for gid, status in self.goal_status.items():
            before = previous.get(gid)
            if status in ACTIVE and before not in ACTIVE:
                self.stop('new Nav2 goal')
            # Goals that finished before this node started arrive with
            # before=None (transient_local replay), so they never trigger.
            # Preempted goals end ABORTED and cancelled ones CANCELED.
            elif status == GoalStatus.STATUS_SUCCEEDED and before in ACTIVE:
                self.state = 'settling'
                self.settle_started = self.now()
                self.get_logger().info('goal reached - waiting for the base to stop')

    def on_stop(self, request, response):
        self.stop('stop requested')
        response.success = True
        response.message = 'stopped'
        return response

    def tick(self):
        if self.state != 'settling':
            return
        now = self.now()
        odom_fresh = self.last_odom is not None and now - self.last_odom < 0.5
        still_for = now - self.still_since if self.still_since else 0.0
        if odom_fresh and still_for >= self.get_parameter('still_time_s').value:
            self.start_policy()
        elif now - self.settle_started > SETTLE_TIMEOUT_S:
            why = 'still moving' if odom_fresh else 'no odometry'
            self.get_logger().error(f'base did not settle ({why}) - policy not started')
            self.state = 'idle'

    def start_policy(self):
        if not self.client.service_is_ready():
            self.get_logger().error('/task/command unavailable - physical_ai_server running?')
            self.state = 'idle'
            return

        req = SendCommand.Request()
        req.command = SendCommand.Request.START_INFERENCE
        req.task_info.policy_path = self.get_parameter('policy_path').value
        req.task_info.task_instruction = [self.get_parameter('instruction').value]
        req.task_info.fps = self.get_parameter('fps').value
        req.task_info.record_inference_mode = False

        self.state = 'starting'
        self.get_logger().info(f'starting policy "{req.task_info.task_instruction[0]}"')
        self.client.call_async(req).add_done_callback(self.on_started)

    def on_started(self, future):
        res = future.result()
        if self.state != 'starting':
            # Stopped while START was in flight - undo it.
            self.send_finish()
        elif res is None or not res.success:
            self.get_logger().error(f'policy start refused: {res.message if res else "no reply"}')
            self.state = 'idle'
        else:
            self.state = 'running'
            self.get_logger().info('policy running')

    def stop(self, why):
        # Only finish what this node started (see module docstring).
        if self.state in ('starting', 'running'):
            self.get_logger().info(f'{why} - stopping policy')
            self.send_finish()
        self.state = 'idle'

    def send_finish(self):
        # FINISH, not STOP: only FINISH clears the server's on_inference flag.
        req = SendCommand.Request()
        req.command = SendCommand.Request.FINISH
        return self.client.call_async(req)


def main():
    # rclpy's own SIGINT handler shuts the context down, and the FINISH below
    # could then not be sent. Python's default one just raises
    # KeyboardInterrupt, which the 20 Hz timer lets spin notice.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PolicyOnArrival()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state in ('starting', 'running'):
            node.get_logger().info('shutting down - stopping policy')
            rclpy.spin_until_future_complete(node, node.send_finish(), timeout_sec=3.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
