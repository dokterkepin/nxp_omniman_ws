#!/usr/bin/env python3
"""
Pick and place with Nav2 and one policy, handing the robot over through the
control lock (control_arbiter).

    acquire "nav" -> drive to pick_area  -> base still -> release "nav"
    visual_align  -> move the base until the cup is in place (holds "align")
    policy_runner -> pick   (holds "policy" until the arm is finished)
    acquire "nav" -> drive to place_area -> base still -> release "nav"
    policy_runner -> place
    acquire "nav" -> drive home          -> release "nav"

This script only ever holds "nav". It asks visual_align to put the cup in
place and policy_runner to run each policy, and waits for each to end - the
runner releases "policy" when the arm reports it is finished - before taking
"nav" again. Acquiring is polite: while someone else holds control, the
mission waits its turn. If control is taken away while
driving (an operator forcing it from the web UI), navigation is cancelled and
the mission stops.

pick_place_shuttle.py is the older fixed-timing mission with base correction.

Places come from config/poses.yaml (saved from the web UI); the policy path and
instructions from config/mission.yaml.

Prereqs:
  - nav2_launch.py, robot localized
  - physical_ai_server_bringup.launch.py
  - control_launch.py on the GPU PC (control_arbiter, policy_runner,
    color_detector, visual_align)

Run:
  ros2 run omniman_vla pick_place_mission.py
  ros2 run omniman_vla pick_place_mission.py --ros-args \\
      -p mission_file:=/path/to/mission.yaml
"""

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# "Still" means every odometry twist component under these for still_time_s.
STILL_LINEAR = 0.01     # m/s
STILL_ANGULAR = 0.02    # rad/s

NAV = 'nav'

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_poses(mission_file):
    """Places from poses.yaml beside the mission file (saved from the web UI)."""
    path = os.path.join(os.path.dirname(os.path.realpath(mission_file)), 'poses.yaml')
    with open(path) as f:
        return yaml.safe_load(f) or {}


def make_pose(node, pose_cfg, frame_id='map'):
    """Planar pose from a {x, y, yaw} dict; yaw in degrees."""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(pose_cfg['x'])
    p.pose.position.y = float(pose_cfg['y'])
    yaw = math.radians(float(pose_cfg['yaw']))
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def call(node, client, request, timeout_s=10.0):
    """Call a service from this single-threaded script; None if no answer."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    return future.result()


class BaseStill:
    """Waits until wheel odometry says the base really stopped.

    Nav2 reports success as soon as the goal checker is satisfied, while the
    base is still coasting. A fixed sleep afterwards is always wrong: too long
    and the mission crawls, too short and the policy starts from a pose that is
    already stale. This waits for the actual stop instead.
    """

    def __init__(self, node):
        self.node = node
        self.still_since = None
        self.last_odom = None
        node.create_subscription(
            Odometry, '/mecanum_drive_controller/odometry', self._on_odom, 10)

    def _on_odom(self, msg):
        t = msg.twist.twist
        now = time.monotonic()
        self.last_odom = now
        moving = (abs(t.linear.x) >= STILL_LINEAR
                  or abs(t.linear.y) >= STILL_LINEAR
                  or abs(t.angular.z) >= STILL_ANGULAR)
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def wait(self, still_time_s, timeout_s):
        """True once the base has been still for still_time_s."""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.monotonic()
            fresh = self.last_odom is not None and now - self.last_odom < 0.5
            if fresh and self.still_since and now - self.still_since >= still_time_s:
                self.node.get_logger().info(f'   base still after {now - start:.2f}s')
                return True
        why = 'still moving' if self.last_odom else 'no odometry'
        self.node.get_logger().warn(f'   base did not settle within {timeout_s:.0f}s ({why})')
        return False


class Control:
    """This script's side of the control lock: acquire, release, watch."""

    def __init__(self, node, owner):
        self.node = node
        self.owner = owner
        self.current = ''
        node.create_subscription(ControlOwner, '/control/owner', self._on_owner, LATCHED)
        self.acquire_client = node.create_client(AcquireControl, '/control/acquire')
        self.release_client = node.create_client(ReleaseControl, '/control/release')

    def _on_owner(self, msg):
        self.current = msg.owner

    def holds(self):
        return self.current == self.owner

    def acquire(self):
        """Wait politely until control is ours. False if the arbiter is gone."""
        log = self.node.get_logger()
        last_note = 0.0
        while True:
            req = AcquireControl.Request()
            req.owner = self.owner
            req.node = self.node.get_fully_qualified_name()
            res = call(self.node, self.acquire_client, req)
            if res is None:
                log.error('control_arbiter not answering - is control_launch.py running?')
                return False
            if res.success:
                break
            if time.monotonic() - last_note > 5.0:
                log.info(f'   waiting for control: {res.message}')
                last_note = time.monotonic()
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)

        # Wait for /control/owner to say so too, or holds() would read stale.
        end = time.monotonic() + 3.0
        while not self.holds() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return True

    def release(self):
        req = ReleaseControl.Request()
        req.owner = self.owner
        call(self.node, self.release_client, req)


class Policy:
    """Asks policy_runner to run a policy, then waits for the run to end."""

    def __init__(self, node):
        self.node = node
        self.status = ''
        node.create_subscription(String, '/policy_runner/status', self._on_status, LATCHED)
        self.run_client = node.create_client(RunPolicy, '/policy_runner/run')

    def _on_status(self, msg):
        self.status = msg.data

    def run(self, path, instruction, label):
        log = self.node.get_logger()
        req = RunPolicy.Request()
        req.policy_path = path
        req.instruction = instruction
        req.force = False
        log.info(f'policy -> {label} ("{instruction}")')
        res = call(self.node, self.run_client, req, timeout_s=20.0)
        if res is None:
            log.error('policy_runner not answering - is control_launch.py running?')
            return False
        if not res.success:
            log.error(f'   {label} refused: {res.message}')
            return False

        # The runner reports starting -> working -> idle; idle after having
        # been busy is the end of this run.
        busy_seen = False
        start = time.monotonic()
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.status != 'idle':
                busy_seen = True
            elif busy_seen or time.monotonic() - start > 5.0:
                break
        log.info(f'   {label} done after {time.monotonic() - start:.1f}s')
        return True


class Align:
    """Asks visual_align to centre the cup, then waits for aligned / failed."""

    def __init__(self, node):
        self.node = node
        self.status = ''
        node.create_subscription(String, '/visual_align/status', self._on_status, LATCHED)
        self.run_client = node.create_client(Trigger, '/visual_align/run')

    def _on_status(self, msg):
        self.status = msg.data

    def run(self):
        log = self.node.get_logger()
        log.info('align -> centre the cup')
        res = call(self.node, self.run_client, Trigger.Request())
        if res is None:
            log.error('visual_align not answering - is control_launch.py running?')
            return False
        if not res.success:
            log.error(f'   align refused: {res.message}')
            return False

        # searching / aligning -> aligned | failed: ...; the result stays
        # latched, so only a result after having been busy is this run's -
        # unless the run was so short that the busy states were missed.
        busy_seen = False
        start = time.monotonic()
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            done = self.status == 'aligned' or self.status.startswith('failed')
            if not done:
                busy_seen = True
            elif busy_seen or time.monotonic() - start > 3.0:
                break
        ok = self.status == 'aligned'
        took = time.monotonic() - start
        if ok:
            log.info(f'   aligned after {took:.1f}s')
        else:
            log.error(f'   align {self.status} after {took:.1f}s')
        return ok


class Mission:

    def __init__(self, nav, cfg):
        self.nav = nav
        self.poses = cfg['poses']
        self.policy_cfg = cfg['policies']['manipulate']
        self.settings = cfg['settings']
        self.still = BaseStill(nav)
        self.control = Control(nav, NAV)
        self.policy = Policy(nav)
        self.align = Align(nav)

    def drive(self, place):
        """Hold "nav" for one leg: drive, wait for the base to stop, release."""
        log = self.nav.get_logger()
        if not self.control.acquire():
            return False
        pose = self.poses[place]
        log.info(f'nav -> {place} (x={pose["x"]:.2f}, y={pose["y"]:.2f}, yaw={pose["yaw"]:.0f})')
        self.nav.goToPose(make_pose(self.nav, pose))

        last = 0.0
        while not self.nav.isTaskComplete():
            if not self.control.holds():
                self.nav.cancelTask()
                log.error(f'   control taken by {self.control.current or "a force release"} '
                          '- navigation cancelled')
                return False
            fb = self.nav.getFeedback()
            if fb and time.time() - last > 1.0:
                log.info(f'   {fb.distance_remaining:.2f} m remaining')
                last = time.time()

        ok = self.nav.getResult() == TaskResult.SUCCEEDED
        if ok:
            log.info(f'   arrived at {place}')
            ok = self.still.wait(float(self.settings['still_time_s']),
                                 float(self.settings['settle_timeout_s']))
        else:
            log.error(f'   nav to {place} FAILED')
        self.control.release()
        return ok

    def align_cup(self):
        """Centre the cup before picking; skipped if align_before_pick is off."""
        if not self.settings.get('align_before_pick', False):
            return True
        if not self.align.run():
            return False
        return self.still.wait(float(self.settings['still_time_s']),
                               float(self.settings['settle_timeout_s']))

    def run(self):
        path = self.policy_cfg['path']
        steps = [
            lambda: self.drive('pick_area'),
            self.align_cup,
            lambda: self.policy.run(path, self.policy_cfg['instruction_pick'], 'pick'),
            lambda: self.drive('place_area'),
            lambda: self.policy.run(path, self.policy_cfg['instruction_place'], 'place'),
            lambda: self.drive('home'),
        ]
        return all(step() for step in steps)

    def shutdown(self):
        """Never leave the robot driving or holding "nav"."""
        self.nav.cancelTask()
        if self.control.holds():
            self.control.release()


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='pick_place_mission')

    default_cfg = f"{get_package_share_directory('omniman_vla')}/config/mission.yaml"
    nav.declare_parameter('mission_file', default_cfg)
    mission_file = nav.get_parameter('mission_file').value
    with open(mission_file) as f:
        cfg = yaml.safe_load(f)
    cfg['poses'] = load_poses(mission_file)
    nav.get_logger().info(f'mission: {mission_file}')

    # BasicNavigator.initial_pose defaults to a zero-norm quaternion, and
    # waitUntilNav2Active() publishes THAT to /initialpose until it hears back on
    # /amcl_pose - so skipping this clobbers a good AMCL estimate with garbage.
    # Assumes the robot is physically at home when launched.
    nav.setInitialPose(make_pose(nav, cfg['poses']['home']))
    nav.get_logger().info('waiting for Nav2...')
    nav.waitUntilNav2Active()

    mission = Mission(nav, cfg)
    try:
        if mission.run():
            nav.get_logger().info('mission complete')
        else:
            nav.get_logger().error('mission ABORTED')
    except KeyboardInterrupt:
        nav.get_logger().warn('interrupted - a running policy keeps going; '
                              'stop it with: ros2 service call /policy_runner/stop '
                              'std_srvs/srv/Trigger')
    finally:
        mission.shutdown()
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
