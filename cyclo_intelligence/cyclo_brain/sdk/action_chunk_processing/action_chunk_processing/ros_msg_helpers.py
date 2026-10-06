"""Shared ROS2 message construction helpers for the policy runtime."""

from typing import Mapping, Sequence

import numpy as np


def make_joint_trajectory(
    classes: Mapping[str, type],
    joint_names: Sequence[str],
    positions: np.ndarray,
):
    """Build a ros2_control-compatible JointTrajectory header + points."""
    JointTrajectoryPoint = classes["JointTrajectoryPoint"]
    Header = classes["Header"]
    Time = classes["Time"]
    Duration = classes["Duration"]

    empty = np.zeros(0, dtype=np.float64)
    point = JointTrajectoryPoint(
        positions=np.asarray(positions, dtype=np.float64),
        velocities=empty,
        accelerations=empty,
        effort=empty,
        time_from_start=Duration(sec=0, nanosec=0),
    )
    header = Header(stamp=Time(sec=0, nanosec=0), frame_id="")
    return header, [point]
