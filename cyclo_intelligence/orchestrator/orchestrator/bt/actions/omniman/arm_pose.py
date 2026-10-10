#!/usr/bin/env python3
"""OmnimanArmPose: move the arm to a pose in omniman_vla/config/arm_poses.yaml."""

import time

from builtin_interfaces.msg import Duration
from omniman_interfaces.srv import AcquireControl, ReleaseControl
from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.actions.omniman.common import (
    ARM, ARM_JOINTS, ARM_TOPIC, GRIPPER_JOINT, REFERENCE_AT_TARGET, RUNNING, SUCCESS,
    _read_yaml, _Step, _text
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class OmnimanArmPose(_Step, BaseAction):
    """Move the arm to `pose` (arm_poses.yaml: ready1, ready2, ready3) and wait
    until it has FINISHED: the arm controller says its trajectory has reached
    the target (/arm_controller/controller_state) and every arm joint is within
    arm_tolerance (mission.yaml) of it. The gripper stays where it is.

    A move takes control as "arm" (like OmnimanNavigate as "nav"): while the
    arm is on its way nothing else - visual_align, a policy, Nav2 - can start,
    and the lock is given back only once the arm has finished. If the arm is
    already there nothing is sent and no lock is taken. Held where it is if the
    tree stops it. `move_s` / `timeout_s` 0 = the arm_move_s / arm_timeout_s
    settings of mission.yaml."""

    def __init__(self, node, pose: str = 'ready1', move_s: float = 0.0,
                 timeout_s: float = 0.0):
        super().__init__(node, name='OmnimanArmPose')
        self._start(node)
        self.pose = _text(pose)
        self.move_s = float(move_s)
        self.timeout_s = float(timeout_s)
        self.targets = _read_yaml('arm_poses.yaml')
        self._clear()

    def _clear(self):
        self.phase, self.future, self.release_future = 'check', None, None
        self.since, self.noted, self.seen_owner = time.monotonic(), 0.0, False
        self.note = ''

    def _send_to(self, positions):
        """One trajectory point for all the controller's joints, the gripper at
        its current position."""
        om = self.om
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS + [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [float(positions[j]) for j in ARM_JOINTS] + [
            float(om.joints.get(GRIPPER_JOINT, 0.0))]
        seconds = om.setting('arm_move_s', self.move_s)
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        msg.points = [point]
        om.arm_pub.publish(msg)

    def tick(self):
        om, now = self.om, time.monotonic()
        target = self.targets.get(self.pose)
        if target is None:
            return self.fail(f'no arm pose "{self.pose}" in arm_poses.yaml')
        missing = [j for j in ARM_JOINTS if j not in target]
        if missing:
            return self.fail(f'arm pose "{self.pose}" in arm_poses.yaml lacks {missing}')
        if self.phase == 'release':
            # The lock must be given back before this node ends, or the next one
            # (visual_align, the policy) could ask for it too early.
            if not self.release_future.done():
                return RUNNING
            self.phase = 'done'
            return self.succeed(f'arm at {self.pose} after {now - self.since:.1f}s')
        if any(j not in om.joints for j in ARM_JOINTS + [GRIPPER_JOINT]):
            if now - self.since > 10.0:
                return self.fail('no arm joints on /joint_states')
            return RUNNING
        tolerance = om.setting('arm_tolerance')
        worst = max(abs(om.joints[j] - float(target[j])) for j in ARM_JOINTS)

        if self.phase == 'check':
            if worst <= tolerance:
                self.phase = 'done'
                return SUCCESS          # already there: nothing sent, nothing logged
            self.phase, self.since = 'acquire', now

        if self.phase == 'acquire':
            if self.future is None:
                req = AcquireControl.Request()
                req.owner = ARM
                req.node = self.node.get_fully_qualified_name()
                self.future = self.call(om.acquire, req)
                if self.future == 'gone':
                    self.phase = 'done'
                    return self.fail('control_arbiter not answering (/control/acquire)')
                return RUNNING
            if not self.future.done():
                return RUNNING
            res, self.future = self.future.result(), None
            if res is None or not res.success:
                if now - self.noted > 5.0:      # someone else has it: wait, ask again
                    self.log_info(f'waiting for control: {res.message if res else "no answer"}')
                    self.noted = now
                return RUNNING
            self.phase, self.since = 'move', now
            self._send_to(target)
            self.log_info(f'arm to {self.pose}: moving on {ARM_TOPIC} '
                          f'({om.setting("arm_move_s", self.move_s):g}s, '
                          f'worst joint {worst:.3f} rad)')
            return RUNNING

        # phase 'move'
        if om.owner == ARM:
            self.seen_owner = True
        elif self.seen_owner:
            self.phase = 'done'
            return self.fail(f'control taken by {om.owner or "a force release"}')
        waited = now - self.since
        reference, error = om.ctrl_reference, om.ctrl_error
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
            om.arm_arrived_at = now
            req = ReleaseControl.Request()
            req.owner = ARM
            self.release_future = om.release_client.call_async(req)
            self.phase = 'release'
            self.log_info(f'arm to {self.pose}: finished after {waited:.1f}s ({detail})')
            return RUNNING
        timeout_s = om.setting('arm_timeout_s', self.timeout_s)
        if waited > timeout_s:
            om.release(ARM)
            self.phase = 'done'
            return self.fail(f'not finished after {waited:.0f}s: {detail}; tolerance '
                             f'{tolerance} - is teleop_bridges_launch.py running?')
        return RUNNING

    def reset(self):
        super().reset()
        om = self.om
        if self.phase == 'acquire' and self.future not in (None, 'gone') \
                and not self.future.done():
            # A "yes" still on its way must not leave the lock held.
            self.future.add_done_callback(
                lambda f: f.result() is not None and f.result().success and om.release(ARM))
        if self.phase in ('move', 'release'):
            if all(j in om.joints for j in ARM_JOINTS):
                self._send_to(om.joints)            # stop where it is
            om.release(ARM)
            self.log_warn(f'arm to {self.pose}: stopped - arm held where it is')
        self._clear()
