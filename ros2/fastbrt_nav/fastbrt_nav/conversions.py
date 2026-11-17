"""
Message <-> numpy conversions for the navigation node.  No rclpy import, so
these are unit-tested without ROS.

Frames.  Gazebo's diff-drive odometry starts at the spawn pose: its "odom"
frame is the world frame moved to (spawn_x, spawn_y, spawn_yaw).  The map,
the goal and the robot profile all use world coordinates, so odometry poses
are mapped with ``odom_to_world``.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["yaw_from_quaternion", "odom_to_world", "scan_arrays", "occupancy_data", "segments_points"]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odom_to_world(pose_odom, spawn) -> np.ndarray:
    """(x, y, theta) in the odom frame -> world frame, given the spawn pose (x, y, yaw)."""
    x, y, th = pose_odom
    sx, sy, syaw = spawn
    c, s = math.cos(syaw), math.sin(syaw)
    return np.array([sx + c * x - s * y, sy + s * x + c * y, (th + syaw + math.pi) % (2 * math.pi) - math.pi])


def scan_arrays(angle_min: float, angle_increment: float, ranges, range_min: float, range_max: float,
                cutoff: float):
    """(angles relative to the sensor heading, ranges) for ``integrate_scan``.

    Follows REP 117 and cuts the range at ``cutoff`` (the profile's sensing
    range): a reading at or beyond min(range_max, cutoff) becomes +inf (free up
    to the cut-off), nan / -inf / below range_min stay invalid."""
    r = np.asarray(ranges, float)
    angles = angle_min + angle_increment * np.arange(len(r))
    limit = min(range_max, cutoff)
    out = r.copy()
    out[np.isposinf(r) | (r >= limit)] = np.inf
    out[np.isnan(r) | np.isneginf(r) | (r < range_min)] = np.nan
    return angles, out


def occupancy_data(state: np.ndarray, unknown: int, free: int, occupied: int) -> list:
    """nav_msgs/OccupancyGrid data (row-major from the lower-left cell):
    -1 unknown, 0 free, 100 occupied."""
    d = np.full(state.shape, -1, np.int8)
    d[state == free] = 0
    d[state == occupied] = 100
    return d.reshape(-1).tolist()


def segments_points(segments) -> list:
    """[(a, b), ...] -> flat [(x, y), ...] for a LINE_LIST marker."""
    pts = []
    for a, b in segments:
        pts += [(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))]
    return pts
