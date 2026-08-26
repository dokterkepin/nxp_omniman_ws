#!/usr/bin/env python3
"""
Mask the robot's own structure out of the laser scan.

WHY THIS EXISTS
---------------
The lidar sees part of the robot itself. Measured on the real robot (40 scans,
stationary) there are two rigid clusters of returns:

    bearing -170 deg .. -158 deg   at ~0.186 m
    bearing +144 deg .. +162 deg   at ~0.212 m

The lidar is mounted at (-0.070, 0.000, 0.167) in base_footprint with yaw 0,
so those land at roughly x = -0.25 m in base_footprint - about 7 cm BEHIND the
declared footprint rear edge (-0.18), which is why footprint_clearing_enabled
does not remove them. Two clusters flanking a clear gap at +/-180 deg is the
signature of robot structure, not a wall.

The previous workaround was obstacle_min_range/raytrace_min_range = 0.45 in the
costmaps, which threw away EVERY return closer than 0.45 m. Since the footprint
extends 0.24 m forward, that blinded the robot in the 0.24-0.45 m ring - exactly
where a person stepping in front of it would be. nav2_collision_monitor is worse
still: its scan source has no range filtering at all, so those self-hits would
sit inside any stop polygon and latch a permanent emergency stop.

WHAT IT DOES
------------
Drops a return only if it is BOTH inside one of the masked bearing sectors AND
closer than max_self_range. A real obstacle further away in the same sector is
passed through untouched, so the rear is not blinded.

Masked returns are set to +inf ("no return") rather than 0.0, so downstream
consumers treat them as empty space rather than as a reading at zero distance.

Run:
  ros2 run omniman_navigation scan_self_filter.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("scan_self_filter")

        # Sectors are [min_deg, max_deg] pairs in the LASER frame, flattened.
        # Measured clusters were -170..-158 and +144..+162; a few degrees of
        # margin on each side absorbs mounting flex and beam spread.
        self.declare_parameter("sectors_deg", [-174.0, -154.0, 142.0, 166.0])
        self.declare_parameter("max_self_range", 0.25)
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")

        flat = list(self.get_parameter("sectors_deg").value)
        if len(flat) % 2 != 0:
            raise ValueError("sectors_deg must contain an even number of values")
        self.sectors = [
            (math.radians(flat[i]), math.radians(flat[i + 1]))
            for i in range(0, len(flat), 2)
        ]
        self.max_self_range = float(self.get_parameter("max_self_range").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # Sensor data QoS: best-effort, matching how drivers publish scans.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(LaserScan, out_topic, qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, qos)

        self._logged = False
        self.get_logger().info(
            f"Masking returns < {self.max_self_range:.2f} m in sectors "
            + ", ".join(
                f"[{math.degrees(a):.0f}, {math.degrees(b):.0f}]deg"
                for a, b in self.sectors
            )
            + f"  ({in_topic} -> {out_topic})"
        )

    def _in_masked_sector(self, angle):
        """True if angle (rad) falls in any masked sector, handling wraparound."""
        # Normalise to [-pi, pi) so sectors spanning +/-180 compare correctly.
        a = (angle + math.pi) % (2.0 * math.pi) - math.pi
        for lo, hi in self.sectors:
            if lo <= hi:
                if lo <= a <= hi:
                    return True
            else:
                # Sector wraps through +/-180.
                if a >= lo or a <= hi:
                    return True
        return False

    def cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        masked = 0

        for i, r in enumerate(ranges):
            # Leave existing non-returns alone.
            if not math.isfinite(r):
                continue
            if r >= self.max_self_range:
                continue
            if self._in_masked_sector(msg.angle_min + i * msg.angle_increment):
                ranges[i] = float("inf")
                masked += 1

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self.pub.publish(out)

        if not self._logged:
            self.get_logger().info(f"First scan filtered: {masked} self-returns masked.")
            self._logged = True


def main():
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
