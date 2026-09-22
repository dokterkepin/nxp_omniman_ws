#!/usr/bin/env python3
"""
Pick and place as a behaviour tree (py_trees): the same steps as
pick_place_mission.py, plus a grasp check with retry and a guard that the cup
is still held while driving to the place.

    pick and place                       Sequence
     ├─ nav to pick_area                 Navigate
     ├─ pick attempts                    Retry (max_pick_attempts)
     │    └─ attempt                     Sequence
     │         ├─ align to pick_target   Align       (visual_align)
     │         ├─ pick policy            PolicyStep  (policy_runner)
     │         └─ grasp succeeded        Holding     (grasp_monitor)
     ├─ while holding                    EternalGuard (holding)
     │    └─ nav to place_area           Navigate
     ├─ align to place_target            Align       (align_before_place)
     ├─ place policy                     PolicyStep
     └─ nav to home                      Navigate

The pick policy closes the gripper whether or not it got the cup (a cup that
was knocked away, or never reached). grasp_monitor tells the two apart - the
fingers stop against the cup instead of closing fully, and the servo pushes -
and a failed grasp makes Retry run align + pick again instead of driving off
empty. If the cup drops on the way to the place, the guard cancels the drive
and the mission stops.

This is the standard pattern for pick with retry (BehaviorTree.CPP's
RetryUntilSuccessful, as in Nav2's recovery trees; py_trees' Retry and
EternalGuard here), with QT-Opt's style of grasp-success check (gripper not
fully closed). Each step only calls the existing services - control_arbiter,
visual_align, policy_runner - and holds "nav" only while driving, exactly like
pick_place_mission.py.

Places from config/poses.yaml; policy, instructions and settings from
config/mission.yaml. pick_target / place_target must be prompts of the
detector (sam_detector's `prompts`), e.g. ["yellow cup lid", "black square"].

Prereqs:
  - nav2_launch.py, robot localized
  - physical_ai_server_bringup.launch.py
  - control_launch.py on the GPU PC (control_arbiter, policy_runner, detector,
    visual_align, grasp_monitor)
  - py_trees: sudo apt install ros-jazzy-py-trees

Run:
  ros2 run omniman_vla pick_place_bt.py
"""

import time

import py_trees
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from pick_place_mission import load_poses, make_pose
from py_trees.common import Status
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

NAV = 'nav'
TICK_S = 0.1
# How long a step waits for a service to appear before failing.
SERVICE_WAIT_S = 10.0

# "Still" means every odometry twist component under these (as the mission).
STILL_LINEAR = 0.01     # m/s
STILL_ANGULAR = 0.02    # rad/s

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Robot:
    """Everything the steps share: the ROS node, service clients and the
    latest value of each topic they watch."""

    def __init__(self, nav, cfg):
        self.nav = nav
        self.cfg = cfg
        self.owner = ''
        self.policy_status = ''
        self.align_status = ''
        self.holding = False
        self.still_since = None
        self.last_odom = 0.0

        nav.create_subscription(ControlOwner, '/control/owner',
                                lambda m: setattr(self, 'owner', m.owner), LATCHED)
        nav.create_subscription(String, '/policy_runner/status',
                                lambda m: setattr(self, 'policy_status', m.data), LATCHED)
        nav.create_subscription(String, '/visual_align/status',
                                lambda m: setattr(self, 'align_status', m.data), LATCHED)
        nav.create_subscription(Bool, '/gripper/holding',
                                lambda m: setattr(self, 'holding', m.data), LATCHED)
        nav.create_subscription(Odometry, '/mecanum_drive_controller/odometry',
                                self._on_odom, 10)
        self.target_pub = nav.create_publisher(String, '/visual_align/target', LATCHED)

        self.acquire = nav.create_client(AcquireControl, '/control/acquire')
        self.release_client = nav.create_client(ReleaseControl, '/control/release')
        self.align_run = nav.create_client(Trigger, '/visual_align/run')
        self.align_stop = nav.create_client(Trigger, '/visual_align/stop')
        self.policy_run = nav.create_client(RunPolicy, '/policy_runner/run')
        self.policy_stop = nav.create_client(Trigger, '/policy_runner/stop')

    def _on_odom(self, msg):
        t = msg.twist.twist
        now = time.monotonic()
        self.last_odom = now
        moving = (abs(t.linear.x) >= STILL_LINEAR or abs(t.linear.y) >= STILL_LINEAR
                  or abs(t.angular.z) >= STILL_ANGULAR)
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def base_still(self):
        """True once odometry has shown the base stopped for still_time_s."""
        now = time.monotonic()
        return (now - self.last_odom < 0.5 and self.still_since is not None
                and now - self.still_since >= float(self.cfg['settings']['still_time_s']))

    def release(self):
        req = ReleaseControl.Request()
        req.owner = NAV
        self.release_client.call_async(req)


class Step(py_trees.behaviour.Behaviour):
    """A step that starts something on initialise() and follows it on each
    tick; terminate() cancels it when the tree interrupts the step."""

    def __init__(self, name, robot):
        super().__init__(name)
        self.robot = robot
        self.log = robot.nav.get_logger()

    def send(self, client, request):
        """Send a request once the service is there - one sent before it is
        discovered can be lost. None while waiting, 'gone' after
        SERVICE_WAIT_S, else the future."""
        if client.service_is_ready():
            return client.call_async(request)
        if time.monotonic() - self.since > SERVICE_WAIT_S:
            self.log.error(f'{self.name}: {client.srv_name} not answering')
            return 'gone'
        return None

    def settle(self, since):
        """RUNNING until the base is still, FAILURE past settle_timeout_s."""
        if self.robot.base_still():
            return Status.SUCCESS
        if time.monotonic() - since > float(self.robot.cfg['settings']['settle_timeout_s']):
            self.log.warn(f'{self.name}: base did not settle')
            return Status.FAILURE
        return Status.RUNNING


class Navigate(Step):
    """Hold "nav" for one leg: acquire, drive with Nav2, wait until the base
    is still, release. Cancelled (and released) if interrupted or if control
    is taken away."""

    def __init__(self, robot, place):
        super().__init__(f'nav to {place}', robot)
        self.place = place

    def initialise(self):
        self.phase, self.future, self.since, self.noted = 'acquire', None, time.monotonic(), 0.0

    def update(self):
        r = self.robot
        if self.phase == 'acquire':
            if self.future is None:
                req = AcquireControl.Request()
                req.owner = NAV
                req.node = r.nav.get_fully_qualified_name()
                self.future = self.send(r.acquire, req)
                if self.future == 'gone':
                    return Status.FAILURE
            elif self.future.done():
                res, self.future = self.future.result(), None
                if res is not None and res.success:
                    self.phase, self.since = 'owner', time.monotonic()
                elif time.monotonic() - self.noted > 5.0:
                    self.log.info(f'{self.name}: waiting for control '
                                  f'({res.message if res else "no answer"})')
                    self.noted = time.monotonic()
            return Status.RUNNING
        if self.phase == 'owner':
            # /control/owner must say so too, or a takeover check reads stale.
            if r.owner == NAV or time.monotonic() - self.since > 3.0:
                pose = r.cfg['poses'][self.place]
                self.log.info(f'{self.name} (x={pose["x"]:.2f}, y={pose["y"]:.2f}, '
                              f'yaw={pose["yaw"]:.0f})')
                r.nav.goToPose(make_pose(r.nav, pose))
                self.phase = 'drive'
            return Status.RUNNING
        if self.phase == 'drive':
            if r.owner != NAV:
                r.nav.cancelTask()
                self.log.error(f'{self.name}: control taken by {r.owner or "a force release"}')
                self.phase = 'done'
                return Status.FAILURE
            if not r.nav.isTaskComplete():
                return Status.RUNNING
            if r.nav.getResult() != TaskResult.SUCCEEDED:
                self.log.error(f'{self.name}: Nav2 failed')
                r.release()
                self.phase = 'done'
                return Status.FAILURE
            self.phase, self.since = 'settle', time.monotonic()
            return Status.RUNNING
        # settle
        status = self.settle(self.since)
        if status != Status.RUNNING:
            r.release()
            self.phase = 'done'
        return status

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.phase in ('owner', 'drive', 'settle'):
            self.robot.nav.cancelTask()
            self.robot.release()
            self.log.warn(f'{self.name}: interrupted - navigation cancelled')


class Align(Step):
    """visual_align to `target` (a detector prompt), then wait for the base
    to be still. Stopped if interrupted."""

    def __init__(self, robot, target):
        super().__init__(f'align to "{target}"', robot)
        self.target = target

    def initialise(self):
        self.robot.target_pub.publish(String(data=self.target))
        self.future = None
        self.phase, self.busy_seen, self.since = 'call', False, time.monotonic()

    def update(self):
        r = self.robot
        if self.phase == 'call':
            if self.future is None:
                self.future = self.send(r.align_run, Trigger.Request())
                if self.future == 'gone':
                    return Status.FAILURE
                return Status.RUNNING
            if not self.future.done():
                return Status.RUNNING
            res = self.future.result()
            if res is None or not res.success:
                self.log.error(f'{self.name}: refused ({res.message if res else "no answer"})')
                return Status.FAILURE
            self.phase, self.since = 'watch', time.monotonic()
            return Status.RUNNING
        if self.phase == 'watch':
            # aligned / failed stay latched: only a result after having been
            # busy is this run's (or after 3 s, if the busy states were missed).
            s = r.align_status
            done = s == 'aligned' or s.startswith('failed')
            if not done:
                self.busy_seen = True
                return Status.RUNNING
            if not self.busy_seen and time.monotonic() - self.since < 3.0:
                return Status.RUNNING
            if s != 'aligned':
                self.log.error(f'{self.name}: {s}')
                return Status.FAILURE
            self.phase, self.since = 'settle', time.monotonic()
            return Status.RUNNING
        return self.settle(self.since)

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.phase == 'watch':
            self.robot.align_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - align stopped')


class PolicyStep(Step):
    """policy_runner runs the policy until the arm reports it is finished
    (back home, see policy_runner.py). Stopped if interrupted."""

    def __init__(self, robot, label, instruction):
        super().__init__(f'{label} policy', robot)
        self.instruction = instruction

    def initialise(self):
        self.future = None
        self.phase, self.busy_seen, self.since = 'call', False, time.monotonic()
        self.log.info(f'{self.name}: "{self.instruction}"')

    def update(self):
        r = self.robot
        if self.phase == 'call':
            if self.future is None:
                req = RunPolicy.Request()
                req.policy_path = r.cfg['policies']['manipulate']['path']
                req.instruction = self.instruction
                req.force = False
                self.future = self.send(r.policy_run, req)
                if self.future == 'gone':
                    return Status.FAILURE
                return Status.RUNNING
            if not self.future.done():
                return Status.RUNNING
            res = self.future.result()
            if res is None or not res.success:
                self.log.error(f'{self.name}: refused ({res.message if res else "no answer"})')
                return Status.FAILURE
            self.phase, self.since = 'watch', time.monotonic()
            return Status.RUNNING
        # starting -> working -> idle; idle after having been busy ends it.
        if r.policy_status != 'idle':
            self.busy_seen = True
            return Status.RUNNING
        if self.busy_seen or time.monotonic() - self.since > 5.0:
            self.phase = 'done'
            self.log.info(f'{self.name}: done after {time.monotonic() - self.since:.0f}s')
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.phase == 'watch':
            self.robot.policy_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - policy stopped')


class Holding(py_trees.behaviour.Behaviour):
    """Condition: grasp_monitor says the gripper holds something."""

    def __init__(self, robot, name='grasp succeeded'):
        super().__init__(name)
        self.robot = robot

    def update(self):
        if self.robot.holding:
            return Status.SUCCESS
        self.robot.nav.get_logger().warn(f'{self.name}: no - gripper is empty')
        return Status.FAILURE


def build_tree(robot):
    s = robot.cfg['settings']
    m = robot.cfg['policies']['manipulate']

    attempt = []
    if s.get('align_before_pick', True):
        attempt.append(Align(robot, s['pick_target']))
    attempt += [PolicyStep(robot, 'pick', m['instruction_pick']), Holding(robot)]
    pick = py_trees.decorators.Retry(
        'pick attempts', py_trees.composites.Sequence('attempt', memory=True, children=attempt),
        num_failures=int(s['max_pick_attempts']))

    carry = py_trees.decorators.EternalGuard(
        'while holding', Navigate(robot, 'place_area'), condition=lambda: robot.holding)

    steps = [Navigate(robot, 'pick_area'), pick, carry]
    if s.get('align_before_place', False):
        steps.append(Align(robot, s['place_target']))
    steps += [PolicyStep(robot, 'place', m['instruction_place']), Navigate(robot, 'home')]
    return py_trees.composites.Sequence('pick and place', memory=True, children=steps)


def call_sync(nav, client, request, timeout_s=5.0):
    """A blocking service call, for shutdown only."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(nav, future, timeout_sec=timeout_s)
    return future.result()


def main():
    # rclpy's SIGINT handler would shut the context down before the steps
    # could stop what they started; Python's default raises KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator(node_name='pick_place_bt')

    default_cfg = f"{get_package_share_directory('omniman_vla')}/config/mission.yaml"
    nav.declare_parameter('mission_file', default_cfg)
    mission_file = nav.get_parameter('mission_file').value
    with open(mission_file) as f:
        cfg = yaml.safe_load(f)
    cfg['poses'] = load_poses(mission_file)
    log = nav.get_logger()
    log.info(f'mission: {mission_file}')

    # As pick_place_mission.py: assumes the robot is at home when launched.
    nav.setInitialPose(make_pose(nav, cfg['poses']['home']))
    log.info('waiting for Nav2...')
    nav.waitUntilNav2Active()

    robot = Robot(nav, cfg)
    root = build_tree(robot)
    tree = py_trees.trees.BehaviourTree(root)
    log.info('\n' + py_trees.display.unicode_tree(root))

    shown = ''
    try:
        while True:
            start = time.monotonic()
            tree.tick()
            view = py_trees.display.unicode_tree(root, show_status=True)
            if view != shown:
                log.info('\n' + view)
                shown = view
            if root.status in (Status.SUCCESS, Status.FAILURE):
                break
            rclpy.spin_once(nav, timeout_sec=0.0)
            time.sleep(max(0.0, TICK_S - (time.monotonic() - start)))
        if root.status == Status.SUCCESS:
            log.info('mission complete')
        else:
            log.error('mission ABORTED')
    except KeyboardInterrupt:
        log.warn('interrupted - stopping everything')
        root.stop(Status.INVALID)
    finally:
        # Never leave the robot driving, aligning, running a policy or
        # holding "nav"; each stop is a no-op when that part is idle.
        nav.cancelTask()
        call_sync(nav, robot.align_stop, Trigger.Request())
        call_sync(nav, robot.policy_stop, Trigger.Request())
        if robot.owner == NAV:
            req = ReleaseControl.Request()
            req.owner = NAV
            call_sync(nav, robot.release_client, req)
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
