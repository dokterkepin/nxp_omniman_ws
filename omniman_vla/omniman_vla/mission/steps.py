"""The steps a mission is made of: Navigate, Align, PolicyStep, Holding,
and attempts() to try some of them again."""

import math
import os
import time

import py_trees
from omniman_interfaces.srv import AcquireControl, RunPolicy
from nav2_simple_commander.robot_navigator import TaskResult
from py_trees.common import Status
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .robot import NAV, make_pose
from .step import Step


class Navigate(Step):
    """Drive to a place in poses.yaml: acquire "nav", drive with Nav2, wait
    until the base is still, release. Cancelled (and released) if interrupted
    or if control is taken away."""

    def __init__(self, robot, place, settle_timeout_s=None,
                 service_wait_s=None):
        super().__init__(f'nav to {place}', robot)
        self.place = place
        self.times = {'settle_timeout_s': settle_timeout_s, 'service_wait_s': service_wait_s}

    def initialise(self):
        self.phase, self.future, self.since, self.noted = 'acquire', None, time.monotonic(), 0.0
        self.feedback_message = ''

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
                else:
                    self.feedback_message = ('waiting for control: '
                                             f'{res.message if res else "no answer"}')
                    if time.monotonic() - self.noted > 5.0:
                        self.log.info(f'{self.name}: {self.feedback_message}')
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
                self.phase = 'done'
                return self.fail(f'control taken by {r.owner or "a force release"}')
            if not r.nav.isTaskComplete():
                return Status.RUNNING
            result = r.nav.getResult()
            if result != TaskResult.SUCCEEDED:
                r.release()
                self.phase = 'done'
                return self.fail(f'Nav2 did not reach it ({getattr(result, "name", result)})')
            self.phase, self.since = 'settle', time.monotonic()
            return Status.RUNNING
        status = self.settle(self.since)
        if status != Status.RUNNING:
            r.release()
            self.phase = 'done'
        if status == Status.SUCCESS:
            return self.succeed('arrived')
        return status

    def progress(self):
        r = self.robot
        if self.phase == 'acquire':
            return f'/control/acquire: waiting, /control/owner is "{r.owner or "nobody"}"'
        if self.phase == 'owner':
            return f'/control/owner: waiting for it to say "{NAV}"'
        if self.phase == 'drive':
            fb = r.nav.getFeedback()
            left = f'{fb.distance_remaining:.2f} m left' if fb else 'no feedback yet'
            return f'Nav2 navigate_to_pose: {left}'
        vx, vy, wz = r.twist
        return (f'/mecanum_drive_controller/odometry: vx {vx:+.3f} vy {vy:+.3f} m/s, '
                f'wz {wz:+.3f} rad/s (still = under {r.settings["still_linear"]} m/s, '
                f'{r.settings["still_angular"]} rad/s)')

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase in ('owner', 'drive', 'settle'):
            self.robot.nav.cancelTask()
            self.robot.release()
            self.log.warn(f'{self.name}: interrupted - navigation cancelled')


class Align(Step):
    """visual_align to `target` - exactly a prompt of the running detector -
    then wait for the base to be still. Stopped if interrupted."""

    def __init__(self, robot, target, timeout_s=None, settle_timeout_s=None,
                 service_wait_s=None):
        super().__init__(f'align to "{target}"', robot)
        self.target = target
        # timeout_s: give up on visual_align after this long (None = wait for
        # it; visual_align has its own limits in visual_align.yaml).
        self.timeout_s = timeout_s
        self.times = {'settle_timeout_s': settle_timeout_s, 'service_wait_s': service_wait_s}

    def initialise(self):
        self.robot.target_pub.publish(String(data=self.target))
        self.future = None
        self.phase, self.busy_seen, self.since = 'call', False, time.monotonic()
        self.start_pose = self.robot.pose
        self.feedback_message = ''

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
                return self.fail(f'visual_align refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'watch', time.monotonic()
            return Status.RUNNING
        if self.phase == 'watch':
            if self.timeout_s is not None and time.monotonic() - self.since > self.timeout_s:
                r.align_stop.call_async(Trigger.Request())
                self.phase = 'done'
                return self.fail(f'no result from visual_align within {self.timeout_s:.0f}s '
                                 f'(last: {r.align_status()}) - align stopped')
            # aligned / failed stay latched: only a result after having been
            # busy is this run's (or after 3 s, if the busy states were missed).
            s = r.align_status()
            done = s == 'aligned' or s.startswith('failed')
            if not done:
                self.busy_seen = True
                self.feedback_message = s
                return Status.RUNNING
            if not self.busy_seen and time.monotonic() - self.since < 3.0:
                return Status.RUNNING
            if s != 'aligned':
                self.phase = 'done'
                return self.fail(s)
            self.phase, self.took = 'settle', time.monotonic() - self.since
            self.since = time.monotonic()
            return Status.RUNNING
        status = self.settle(self.since)
        if status == Status.SUCCESS:
            (x0, y0, a0), (x1, y1, a1) = self.start_pose, r.pose
            turned = math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
            moved = math.hypot(x1 - x0, y1 - y0)
            self.phase = 'done'
            return self.succeed(f'aligned in {self.took:.1f}s, base turned {turned:+.0f} deg, '
                                f'moved {moved:.2f} m')
        return status

    def progress(self):
        r = self.robot
        waited = time.monotonic() - self.since
        if self.phase == 'call':
            return f'/visual_align/run: calling (target "{self.target}")'
        if self.phase == 'watch':
            return (f'/visual_align/status: {r.align_status() or "nothing yet"} '
                    f'(target "{self.target}", {waited:.0f}s)')
        vx, vy, wz = r.twist
        return (f'/mecanum_drive_controller/odometry: vx {vx:+.3f} vy {vy:+.3f} m/s, '
                f'wz {wz:+.3f} rad/s - waiting for the base to stop')

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase == 'watch':
            self.robot.align_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - align stopped')


class PolicyStep(Step):
    """Run an arm policy through policy_runner until the arm is finished (back
    home, see policy_runner.py). policy_path empty = policy_runner's default.
    Stopped if interrupted."""

    def __init__(self, robot, label, instruction, policy_path='',
                 timeout_s=None, service_wait_s=None):
        super().__init__(f'{label} policy', robot)
        self.instruction = instruction
        self.policy_path = os.path.expanduser(policy_path) if policy_path else ''
        # timeout_s: stop the policy and fail after this long (None = until
        # policy_runner says the arm is finished, or its own run_timeout_s).
        self.timeout_s = timeout_s
        self.times = {'service_wait_s': service_wait_s}

    def initialise(self):
        self.future = None
        self.phase, self.busy_seen, self.since = 'call', False, time.monotonic()
        self.feedback_message = ''
        self.log.info(f'{self.name}: "{self.instruction}"')

    def update(self):
        r = self.robot
        if self.phase == 'call':
            if self.future is None:
                req = RunPolicy.Request()
                req.policy_path = self.policy_path
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
                return self.fail(f'policy_runner refused: {res.message if res else "no answer"}')
            self.phase, self.since = 'watch', time.monotonic()
            return Status.RUNNING
        if self.timeout_s is not None and time.monotonic() - self.since > self.timeout_s:
            r.policy_stop.call_async(Trigger.Request())
            self.phase = 'done'
            return self.fail(f'policy ran longer than {self.timeout_s:.0f}s '
                             f'({r.arm_state()}) - policy stopped')
        # starting -> working -> idle; idle after having been busy ends it.
        if r.policy_status() != 'idle':
            self.busy_seen = True
            self.feedback_message = f'{r.policy_status()} {time.monotonic() - self.since:.0f}s'
            return Status.RUNNING
        if self.busy_seen or time.monotonic() - self.since > 5.0:
            self.phase = 'done'
            return self.succeed(f'arm back home after {time.monotonic() - self.since:.0f}s')
        return Status.RUNNING

    def progress(self):
        r = self.robot
        waited = time.monotonic() - self.since
        if self.phase == 'call':
            return f'/policy_runner/run: calling ("{self.instruction}")'
        return (f'/policy_runner/status: {r.policy_status() or "nothing yet"} | '
                f'{r.task_phase()} | {waited:.0f}s\n'
                f'      arm (/policy_runner/arm): {r.arm_state()}\n'
                f'      gripper (/grasp_monitor/state): {r.gripper_state()}')

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase == 'watch':
            self.robot.policy_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - policy stopped')


class Holding(py_trees.behaviour.Behaviour):
    """Condition on grasp_monitor: SUCCESS when the gripper holds something
    (holding=True: a pick worked) or, with holding=False, when it no longer
    does (a place let go)."""

    def __init__(self, robot, name, holding=True):
        super().__init__(name)
        self.robot = robot
        self.want = holding

    def update(self):
        # grasp_state carries the measured values, e.g.
        # "empty - closed on nothing (position -0.0120, effort -1)".
        self.feedback_message = self.robot.gripper_state()
        if self.robot.is_holding() == self.want:
            self.robot.nav.get_logger().info(f'{self.name}: {self.feedback_message}')
            return Status.SUCCESS
        self.robot.nav.get_logger().warn(f'{self.name}: no - {self.feedback_message}')
        return Status.FAILURE


def attempts(name, children, times):
    """Run `children` in order, and start them again from the first if any of
    them fails - up to `times` failures (py_trees' Retry over a memory
    Sequence). This is how a step that can genuinely fail gets another go:
    a pick that closed on nothing aligns and picks again."""
    return py_trees.decorators.Retry(
        name, py_trees.composites.Sequence('attempt', memory=True, children=children),
        num_failures=int(times))
