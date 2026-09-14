#!/usr/bin/env python3
"""
One owner of the robot at a time.

    /control/owner    omniman_interfaces/msg/ControlOwner    latched
    /control/acquire  omniman_interfaces/srv/AcquireControl
    /control/release  omniman_interfaces/srv/ReleaseControl

A lock and nothing more. It knows nothing about Nav2, policies or tasks -
owners are just names such as "nav" or "policy". Taking part is opt-in: a
program that never calls acquire is not stopped by it. A program that does:

    acquire(owner, node)   refused while someone else holds control
    ...work, watching /control/owner in case control is taken away...
    release(owner)         the "I'm done" the next owner is waiting for

acquire with force=true takes control from the current owner - an operator's
override, not something scripts should do. The previous owner sees
/control/owner change and is expected to stop what it is doing.

If the node holding control disappears (crashed, killed), control is released
after holder_lost_s, so nothing waits forever on an owner that is gone.

Run:
  ros2 run omniman_vla control_arbiter.py
"""

import rclpy
from omniman_interfaces.msg import ControlOwner
from omniman_interfaces.srv import AcquireControl, ReleaseControl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ControlArbiter(Node):

    def __init__(self):
        super().__init__('control_arbiter')
        # How long the holder's node may be missing from the ROS graph before
        # its control is released. Discovery can lag a few seconds, so not less.
        self.declare_parameter('holder_lost_s', 5.0)

        self.owner = ''
        self.node_name = ''
        self.since = self.get_clock().now()
        self.missing_since = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(ControlOwner, '/control/owner', latched)
        self.create_service(AcquireControl, '/control/acquire', self.on_acquire)
        self.create_service(ReleaseControl, '/control/release', self.on_release)
        self.create_timer(1.0, self.check_holder)

        self.publish()
        self.get_logger().info('ready - nobody holds control')

    def publish(self):
        msg = ControlOwner()
        msg.owner = self.owner
        msg.node = self.node_name
        msg.since = self.since.to_msg()
        self.pub.publish(msg)

    def set_owner(self, owner, node_name):
        self.owner = owner
        self.node_name = node_name
        self.since = self.get_clock().now()
        self.missing_since = None
        self.publish()

    def on_acquire(self, request, response):
        response.previous_owner = self.owner
        if not request.owner:
            response.success = False
            response.message = 'owner must not be empty'
        elif self.owner in ('', request.owner):
            self.set_owner(request.owner, request.node)
            response.success = True
            response.message = f'{request.owner} has control'
            holder = request.node or 'no node'
            self.get_logger().info(f'{request.owner} acquired control ({holder})')
        elif request.force:
            previous = self.owner
            self.set_owner(request.owner, request.node)
            response.success = True
            response.message = f'{request.owner} took control from {previous}'
            self.get_logger().warn(f'{request.owner} FORCED control away from {previous}')
        else:
            response.success = False
            response.message = f'control is held by {self.owner}'
        return response

    def on_release(self, request, response):
        if not self.owner:
            response.success = True
            response.message = 'nobody holds control'
        elif request.force or request.owner == self.owner:
            released = self.owner
            self.set_owner('', '')
            response.success = True
            response.message = f'{released} released control'
            how = 'force-released' if request.force and request.owner != released else 'released'
            self.get_logger().info(f'{released} {how} control')
        else:
            response.success = False
            response.message = f'control is held by {self.owner}, not {request.owner}'
        return response

    def check_holder(self):
        """Release control whose holder node has vanished from the graph."""
        if not self.owner or not self.node_name:
            return
        alive = any(
            (ns.rstrip('/') + '/' + name) == self.node_name
            for name, ns in self.get_node_names_and_namespaces())
        now = self.get_clock().now()
        if alive:
            self.missing_since = None
            return
        if self.missing_since is None:
            self.missing_since = now
            return
        gone_s = (now - self.missing_since).nanoseconds * 1e-9
        if gone_s >= self.get_parameter('holder_lost_s').value:
            self.get_logger().error(
                f'{self.owner} holder {self.node_name} is gone - releasing control')
            self.set_owner('', '')


def main():
    rclpy.init()
    node = ControlArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
