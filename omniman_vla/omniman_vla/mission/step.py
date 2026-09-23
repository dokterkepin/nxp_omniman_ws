"""Step: the base class every step is built on. It holds what all steps
share - the fail/succeed reasons shown in the tree, its timings (seconds), waiting for a service
before calling it (send), and waiting for the base to stop (settle).

A new step subclasses Step and fills in initialise / update / terminate
(py_trees' Behaviour), plus progress() for its log line."""

import time

import py_trees
from py_trees.common import Status


class Step(py_trees.behaviour.Behaviour):
    """Base class for a step that drives a building block: start it
    (initialise / the first ticks), follow it (update, every tick), cancel it
    when interrupted (terminate). Subclass it for a new building block."""

    def __init__(self, name, robot):
        super().__init__(name)
        self.robot = robot
        self.times = {}                 # per-step overrides of the settings
        self.log = robot.nav.get_logger()
        self.since = time.monotonic()

    def fail(self, reason):
        """FAILURE, with the reason shown next to the step in the tree."""
        self.feedback_message = reason
        self.log.error(f'{self.name}: {reason}')
        return Status.FAILURE

    def succeed(self, note):
        """SUCCESS, with a note shown next to the step in the tree."""
        self.feedback_message = note
        self.log.info(f'{self.name}: {note}')
        return Status.SUCCESS

    def progress(self):
        """One line about what this step is waiting for, with the topic or
        service it reads - logged every log_every_s while the step runs."""
        return self.feedback_message

    def interrupted(self, new_status):
        """For terminate(): True only if the step was still RUNNING when the
        tree stopped it - terminate() also runs after SUCCESS / FAILURE."""
        return new_status == Status.INVALID and self.status == Status.RUNNING

    def send(self, client, request):
        """Send a request once the service is there - one sent before it is
        discovered can be lost. None while waiting, 'gone' after
        service_wait_s (reason set), else the future."""
        if client.service_is_ready():
            return client.call_async(request)
        if time.monotonic() - self.since > self.seconds('service_wait_s'):
            self.fail(f'{client.srv_name} not answering - is control_launch.py running?')
            return 'gone'
        return None

    def seconds(self, name):
        """A timing for this step: what the mission passed in, else the
        mission file's setting of that name."""
        value = self.times.get(name)
        return float(self.robot.settings[name] if value is None else value)

    def settle(self, since):
        """RUNNING until the base is still, FAILURE past settle_timeout_s."""
        r = self.robot
        if r.base_still():
            return Status.SUCCESS
        if time.monotonic() - since > self.seconds('settle_timeout_s'):
            if time.monotonic() - r.last_odom > 0.5:
                return self.fail('base did not settle: no odometry for '
                                 f'{time.monotonic() - r.last_odom:.1f}s')
            vx, vy, wz = r.twist
            return self.fail(f'base did not settle: odometry still moving (vx {vx:+.3f}, '
                             f'vy {vy:+.3f} m/s, wz {wz:+.3f} rad/s; still = under '
                             f'{r.settings["still_linear"]} m/s and '
                             f'{r.settings["still_angular"]} rad/s)')
        return Status.RUNNING
