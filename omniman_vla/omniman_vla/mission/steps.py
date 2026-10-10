"""The steps a mission is made of: Navigate, Align, ArmPose, PolicyStep,
Holding, together() to run steps at the same time, search_align() to look for
a target from several arm poses, and attempts() to try some of them again."""

import math
import os
import time

import py_trees
from omniman_interfaces.srv import AcquireControl, ReleaseControl, RunPolicy
from nav2_simple_commander.robot_navigator import TaskResult
from py_trees.common import Status
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .policy_info import policy_info
from .robot import ARM, ARM_JOINTS, ARM_TOPIC, GRIPPER_JOINT, NAV, make_pose
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
    """visual_align to `target` - any text, e.g. 'yellow cup lid': the
    detector looks for exactly this - then wait for the base to be still.
    Stopped if interrupted. When it ends the prompt is cleared, so the
    detector goes idle (it only shows the camera picture) until the next one."""

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
        self.active = True              # the detector has our prompt until terminate()
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
            # busy is this run's. Until it has been seen busy, nothing counts.
            s = r.align_status()
            done = s == 'aligned' or s.startswith('failed')
            if not done:
                self.busy_seen = True
                self.feedback_message = s
                return Status.RUNNING
            if not self.busy_seen:
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
        if getattr(self, 'active', False):
            # Done (or stopped): take the prompt away. The detector then stops
            # running the model and only shows the camera picture.
            self.robot.target_pub.publish(String(data=''))
            self.active = False


# The controller's trajectory counts as at its end when its reference is this
# close to the target (rad): it samples the last point exactly.
REFERENCE_AT_TARGET = 1e-3


class ArmPose(Step):
    """Move the arm to a pose in arm_poses.yaml, and wait until it has
    finished: the arm controller says its trajectory has reached the target
    (/arm_controller/controller_state) and every arm joint is within
    arm_tolerance of it. The gripper stays where it is.

    A move takes control as "arm" (like Navigate does as "nav"): while the
    arm is on its way, nothing else - visual_align, a policy, Nav2 - can start,
    and the lock is given back only once the arm has finished. If the arm is
    already there, nothing is sent and no lock is taken. The move is sent on
    /leader/joint_trajectory - the policy's path - so the teleop relay must be
    running. Held where it is if interrupted."""

    def __init__(self, robot, pose, move_s=None, timeout_s=None, service_wait_s=None):
        super().__init__(f'arm to {pose}', robot)
        self.pose = pose
        # move_s: how long the move takes; timeout_s: fail if not there by
        # then (both from mission.yaml unless given here).
        self.times = {'arm_move_s': move_s, 'arm_timeout_s': timeout_s,
                      'service_wait_s': service_wait_s}

    def initialise(self):
        self.phase, self.future, self.since, self.noted = 'check', None, time.monotonic(), 0.0
        self.seen_owner = False         # /control/owner has said "arm" once
        self.release_future = None
        self.feedback_message = ''

    def worst(self, target):
        joints = self.robot.joints
        return max(abs(joints[j] - float(target[j])) for j in ARM_JOINTS)

    def send_to(self, positions):
        """One trajectory point for all the controller's joints, the gripper
        at its current position."""
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS + [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [float(positions[j]) for j in ARM_JOINTS] + [
            float(self.robot.joints.get(GRIPPER_JOINT, 0.0))]
        seconds = self.seconds('arm_move_s')
        point.time_from_start = Duration(sec=int(seconds),
                                         nanosec=int((seconds % 1.0) * 1e9))
        msg.points = [point]
        self.robot.arm_pub.publish(msg)

    def update(self):
        r = self.robot
        target = (r.cfg.get('arm_poses') or {}).get(self.pose)
        if target is None:
            return self.fail(f'no arm pose "{self.pose}" in arm_poses.yaml')
        missing = [j for j in ARM_JOINTS if j not in target]
        if missing:
            return self.fail(f'arm pose "{self.pose}" in arm_poses.yaml lacks {missing}')
        if self.phase == 'release':
            # The lock must be given back before this step ends, or the next
            # one (visual_align, the policy) could ask for it too early.
            if not self.release_future.done():
                return Status.RUNNING
            return self.succeed(f'there after {time.monotonic() - self.since:.1f}s')
        if any(j not in r.joints for j in ARM_JOINTS + [GRIPPER_JOINT]):
            if time.monotonic() - self.since > self.seconds('service_wait_s'):
                return self.fail('no arm joints on /joint_states')
            self.feedback_message = 'waiting for /joint_states'
            return Status.RUNNING
        tolerance = float(r.settings['arm_tolerance'])
        worst = self.worst(target)

        if self.phase == 'check':
            if worst <= tolerance:
                # Said nothing in the log: search_align asks for the same pose
                # several times, and nothing was done.
                self.feedback_message = f'already there (worst joint {worst:.3f} rad)'
                return Status.SUCCESS
            self.phase, self.since = 'acquire', time.monotonic()

        if self.phase == 'acquire':
            if self.future is None:
                req = AcquireControl.Request()
                req.owner = ARM
                req.node = r.nav.get_fully_qualified_name()
                self.future = self.send(r.acquire, req)
                if self.future == 'gone':
                    self.phase = 'done'
                    return Status.FAILURE
                return Status.RUNNING
            if not self.future.done():
                return Status.RUNNING
            res, self.future = self.future.result(), None
            if res is None or not res.success:
                answer = res.message if res else 'no answer'
                self.feedback_message = f'waiting for control: {answer}'
                if time.monotonic() - self.noted > 5.0:
                    self.log.info(f'{self.name}: {self.feedback_message}')
                    self.noted = time.monotonic()
                return Status.RUNNING
            self.phase = 'move'
            self.send_to(target)
            self.since = time.monotonic()
            self.log.info(f'{self.name}: moving on {ARM_TOPIC} '
                          f'({self.seconds("arm_move_s"):g}s, worst joint {worst:.3f} rad)')
            return Status.RUNNING

        # phase 'move'
        if r.owner == ARM:
            self.seen_owner = True
        elif self.seen_owner:
            self.phase = 'done'
            return self.fail(f'control taken by {r.owner or "a force release"}')
        waited = time.monotonic() - self.since
        reference, error = r.ctrl_reference, r.ctrl_error
        if all(j in reference and j in error for j in ARM_JOINTS):
            behind = max(abs(reference[j] - float(target[j])) for j in ARM_JOINTS)
            tracking = max(abs(error[j]) for j in ARM_JOINTS)
            finished = (behind <= REFERENCE_AT_TARGET and tracking <= tolerance
                        and worst <= tolerance)
            detail = (f'trajectory {behind:.3f} rad from the target, tracking error '
                      f'{tracking:.3f} rad, worst joint {worst:.3f} rad')
        else:
            finished = False
            detail = 'no reading yet from /arm_controller/controller_state'
        if finished:
            r.arm_arrived_at = r.now()
            self.release_future = r.release_client.call_async(self.release_request())
            self.phase = 'release'
            self.feedback_message = 'finished - giving control back'
            self.log.info(f'{self.name}: finished after {waited:.1f}s ({detail})')
            return Status.RUNNING
        if waited > self.seconds('arm_timeout_s'):
            r.release(ARM)
            self.phase = 'done'
            return self.fail(f'not finished after {waited:.0f}s: {detail}; tolerance '
                             f'{tolerance} - is teleop_bridges_launch.py running?')
        self.feedback_message = f'{detail}, {waited:.0f}s'
        return Status.RUNNING

    def release_request(self):
        req = ReleaseControl.Request()
        req.owner = ARM
        return req

    def progress(self):
        r = self.robot
        if self.phase == 'acquire':
            return f'/control/acquire: waiting, /control/owner is "{r.owner or "nobody"}"'
        if self.phase == 'release':
            return '/control/release: giving control back'
        if self.phase == 'move':
            return f'/joint_states: {self.feedback_message} (going to "{self.pose}")'
        return f'/joint_states: {self.feedback_message or "reading the arm"}'

    def terminate(self, new_status):
        if not self.interrupted(new_status):
            return
        if self.phase == 'acquire' and self.future not in (None, 'gone') \
                and not self.future.done():
            # A "yes" still on its way must not leave the lock held.
            self.future.add_done_callback(
                lambda f: f.result() is not None and f.result().success
                and self.robot.release(ARM))
        if self.phase in ('move', 'release'):
            if all(j in self.robot.joints for j in ARM_JOINTS):
                self.send_to(self.robot.joints)       # stop where it is
            self.robot.release(ARM)
            self.log.warn(f'{self.name}: interrupted - arm held where it is')


class Look(Step):
    """Look for `target` from where the arm is now, without turning the base:
    SUCCESS as soon as the detector reports it, FAILURE once it has reported
    on look_frames frames (mission.yaml) without finding it. The prompt is
    cleared when it ends, so the detector goes idle. Use it to see whether
    a pose can see the target before spending a turn on it."""

    def __init__(self, robot, target, frames=None, service_wait_s=None):
        super().__init__(f'look for "{target}"', robot)
        self.target = target
        self.frames = frames
        self.times = {'service_wait_s': service_wait_s}

    def initialise(self):
        r = self.robot
        r.target_pub.publish(String(data=self.target))
        self.active = True
        self.seen0, self.hits0, self.since = r.detections_seen, r.detection_hits, time.monotonic()
        self.feedback_message = ''

    def update(self):
        r = self.robot
        frames = r.detections_seen - self.seen0
        limit = int(r.settings['look_frames'] if self.frames is None else self.frames)
        if r.detection_hits > self.hits0 and r.last_hit[0] == self.target:
            return self.succeed(f'seen from here (score {r.last_hit[1]:.2f}, '
                                f'{frames} detector frames)')
        if frames >= limit:
            return self.fail(f'not seen from here ({frames} detector frames)')
        if frames == 0 and time.monotonic() - self.since > self.seconds('service_wait_s'):
            return self.fail('no detections from /sam_detector/detections - is it running?')
        self.feedback_message = f'{frames} of {limit} detector frames, nothing yet'
        return Status.RUNNING

    def progress(self):
        return f'/sam_detector/detections: {self.feedback_message}'

    def terminate(self, new_status):
        if getattr(self, 'active', False):
            self.robot.target_pub.publish(String(data=''))
            self.active = False


def together(name, steps):
    """Run `steps` at the same time. Succeeds once every one has; if one
    fails, the others are stopped and this fails.

        together('drive, arm to ready1',
                 [Navigate(robot, 'pick_area'), ArmPose(robot, 'ready1')])

    A step that has finished is not run again while the others go on."""
    return py_trees.composites.Parallel(
        name, policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True),
        children=list(steps))


def search_align(robot, target, poses=('ready1', 'ready2', 'ready3'), timeout_s=None):
    """Find `target` from the arm poses, then align to it.

    First, without turning: for each pose, move the arm, Look, and - if the
    target is seen - Align (which then finds it at once). A few seconds per
    pose. Only if no pose sees it from where the base stands, turn: for each
    pose, move the arm and Align, which turns the base looking for the target
    (search_turns in visual_align.yaml). Then the arm goes back to the first
    pose, where the policy starts. timeout_s is each Align's.

        search_align(robot, 'yellow cup lid', timeout_s=60)
    """
    def sequence(name, children):
        return py_trees.composites.Sequence(name, memory=True, children=children)

    seen = [sequence(f'see from {pose}', [ArmPose(robot, pose), Look(robot, target),
                                          Align(robot, target, timeout_s=timeout_s)])
            for pose in poses]
    turned = [sequence(f'turn from {pose}', [ArmPose(robot, pose),
                                             Align(robot, target, timeout_s=timeout_s)])
              for pose in poses]
    find = py_trees.composites.Selector(f'find "{target}"', memory=True, children=[
        py_trees.composites.Selector('without turning', memory=True, children=seen),
        py_trees.composites.Selector('by turning', memory=True, children=turned)])
    return sequence(f'search and align to "{target}"', [find, ArmPose(robot, poses[0])])


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
        self.phase, self.busy_seen, self.since = 'fresh', False, time.monotonic()
        self.feedback_message = ''
        self.log.info(f'{self.name}: "{self.instruction}"')
        for line in policy_info(self.policy_path):
            self.log.info(f'{self.name}: {line}')

    def update(self):
        r = self.robot
        if self.phase == 'fresh':
            # Never start on a picture from before the robot settled: wait for a
            # camera frame that arrives after the base stopped and the arm
            # finished. (Judged by when frames arrive here, not by their header
            # stamps, which are wrong.)
            settled = r.settled_at()
            if settled is None:
                self.feedback_message = 'waiting for the base to be still'
            elif r.camera_at < settled:
                self.feedback_message = 'waiting for a camera frame after the robot settled'
            else:
                waited = time.monotonic() - self.since
                self.log.info(f'{self.name}: waited {waited:.1f}s for the base, the arm and '
                              'a camera frame - starting' if waited >= 0.05 else
                              f'{self.name}: base, arm and camera were already settled - '
                              'starting')
                self.phase, self.since = 'call', time.monotonic()
            if self.phase == 'fresh':
                if self.timeout_s is not None and time.monotonic() - self.since > self.timeout_s:
                    return self.fail(f'no camera frame from after the robot settled within '
                                     f'{self.timeout_s:.0f}s ({self.feedback_message}) - '
                                     'is /image_raw/compressed arriving?')
                return Status.RUNNING
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
            self.left_home = False
            return Status.RUNNING
        # policy_runner says where the arm is every 0.5 s while it runs; only a
        # reading from this run counts (the topic is latched).
        if (not self.left_home and r.arm_state_at > self.since
                and r.arm_state().startswith('away')):
            self.left_home = True
            self.log.info(f'{self.name}: arm left home after '
                          f'{time.monotonic() - self.since:.1f}s')
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
        if self.busy_seen:
            self.phase = 'done'
            return self.succeed(f'arm back home after {time.monotonic() - self.since:.0f}s')
        return Status.RUNNING

    def progress(self):
        r = self.robot
        waited = time.monotonic() - self.since
        if self.phase == 'fresh':
            return f'/image_raw/compressed: {self.feedback_message}'
        if self.phase == 'call':
            return f'/policy_runner/run: calling ("{self.instruction}")'
        # Only the two things that decide the step: is the arm home, is
        # the gripper holding. The full readings: robot.arm_state(),
        # robot.gripper_state().
        arm = r.arm_state().split(':')[0].split(',')[0]
        gripper = r.gripper_state().split(' (')[0]
        return f'arm {arm} | gripper {gripper} | {waited:.0f}s'

    def terminate(self, new_status):
        if self.interrupted(new_status) and self.phase == 'watch':
            self.robot.policy_stop.call_async(Trigger.Request())
            self.log.warn(f'{self.name}: interrupted - policy stopped')


class Holding(py_trees.behaviour.Behaviour):
    """Condition on grasp_monitor: SUCCESS when the gripper holds something
    (holding=True: a pick worked) or, with holding=False, when it no longer
    does (a place let go).

    grasp_monitor only changes its answer once a reading has held for
    stable_s, so right after a policy ends it can still be one step behind a
    gripper that is still closing or opening. This waits until grasp_monitor
    agrees with what the gripper joint reads right now, then judges - it fails
    only if they never agree within holding_timeout_s (mission.yaml)."""

    def __init__(self, robot, name, holding=True, timeout_s=None):
        super().__init__(name)
        self.robot = robot
        self.want = holding
        self.timeout_s = timeout_s
        self.since = time.monotonic()

    def initialise(self):
        self.since = time.monotonic()

    def update(self):
        r, log = self.robot, self.robot.nav.get_logger()
        reads = r.gripper_reads_holding()
        if reads is None or reads != r.is_holding():
            waited = time.monotonic() - self.since
            limit = float(r.settings['holding_timeout_s'] if self.timeout_s is None
                          else self.timeout_s)
            self.feedback_message = (
                'waiting for /joint_states' if reads is None else
                f'the gripper reads {"holding" if reads else "not holding"} but grasp_monitor '
                f'says {r.gripper_state()}')
            if waited > limit:
                log.error(f'{self.name}: no - grasp_monitor did not agree with the gripper '
                          f'within {limit:g}s ({self.feedback_message})')
                return Status.FAILURE
            return Status.RUNNING
        # grasp_state carries the measured values, e.g.
        # "empty - closed on nothing (position -0.0120, effort -1)".
        self.feedback_message = r.gripper_state()
        if r.is_holding() == self.want:
            log.info(f'{self.name}: {self.feedback_message}')
            return Status.SUCCESS
        log.warn(f'{self.name}: no - {self.feedback_message}')
        return Status.FAILURE


def task(name, steps, attempts=1, on_failure=None):
    """One state of a mission: its steps, how many tries, and what to run if
    it still fails.

        task('pick',
             steps=[Align(robot, 'cup'), PolicyStep(robot, 'pick', 'pick it'),
                    Holding(robot, 'grasp succeeded', holding=True)],
             attempts=3,
             on_failure=[Navigate(robot, 'home')])

    The steps run in order; if one fails they start again from the first,
    up to `attempts` tries in all. If every try fails, the `on_failure` steps
    run - any steps: drive somewhere, run a policy, several of them:
      - they succeed    the task counts as done, the mission carries on
      - one fails       the task fails, the mission starts again
    End on_failure with start_again() to always start the mission again
    after it, e.g. on_failure=[Navigate(robot, 'home'), start_again()]."""
    tries = py_trees.decorators.Retry(
        f'{name} ({attempts} tries)',
        py_trees.composites.Sequence(name, memory=True, children=list(steps)),
        num_failures=int(attempts))
    if not on_failure:
        return tries
    return py_trees.composites.Selector(
        f'{name}, or on failure', memory=True, children=[
            tries,
            py_trees.composites.Sequence(f'{name} failed', memory=True,
                                         children=list(on_failure)),
        ])


def start_again(reason='start the mission again'):
    """A step that always fails: put it last in on_failure to make the
    mission start again from its first task after the recovery."""
    return py_trees.behaviours.Failure(reason)


def mission(name, tasks, restarts=2):
    """The tasks in order. If one fails (after its own on_failure steps), the
    mission starts again from the first task - up to `restarts` times, then
    it fails."""
    return py_trees.decorators.Retry(
        f'{name} (restarts: {restarts})',
        py_trees.composites.Sequence(name, memory=True, children=list(tasks)),
        num_failures=int(restarts) + 1)


def _as_one(steps, name):
    """A list of steps as one memory Sequence; a single step as itself."""
    if isinstance(steps, (list, tuple)):
        return py_trees.composites.Sequence(name, memory=True, children=list(steps))
    return steps


def on_failure(steps, then, name=None):
    """Run `steps`; if one of them fails, run `then` - whatever the mission
    wants to happen next after that failure (py_trees' Selector).

        on_failure(Align(robot, 'cup'), then=Navigate(robot, 'home'))
        on_failure(Align(robot, 'cup'),
                   then=[Navigate(robot, 'pick_area'), Align(robot, 'cup')])

    `steps` and `then` are each one step or a list of steps. Succeeds if
    `steps` did, or else if `then` did; fails only if both failed. Put a new
    step object in `then` - py_trees allows each object only once in a tree.
    Wrap it in attempts() to repeat the whole thing."""
    first = _as_one(steps, 'try')
    return py_trees.composites.Selector(
        name or f'{first.name}, on failure', memory=True,
        children=[first, _as_one(then, 'on failure')])


def attempts(name, children, times):
    """Run `children` in order, and start them again from the first if any of
    them fails - up to `times` failures (py_trees' Retry over a memory
    Sequence). This is how a step that can genuinely fail gets another go:
    a pick that closed on nothing aligns and picks again."""
    return py_trees.decorators.Retry(
        name, py_trees.composites.Sequence('attempt', memory=True, children=children),
        num_failures=int(times))
