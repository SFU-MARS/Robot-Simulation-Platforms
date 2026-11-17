import math

import numpy as np
import pytest

from fastbrt_nav.conversions import odom_to_world, occupancy_data, scan_arrays, segments_points, yaw_from_quaternion


def test_yaw_from_quaternion():
    for yaw in (0.0, 0.3, -2.0, math.pi / 2):
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        assert yaw_from_quaternion(*q) == pytest.approx(yaw)


def test_odom_to_world():
    spawn = (-0.8, -0.8, 0.2)
    assert odom_to_world((0.0, 0.0, 0.0), spawn) == pytest.approx([-0.8, -0.8, 0.2])
    # one metre straight ahead in the odom frame = along the spawn heading in the world
    assert odom_to_world((1.0, 0.0, 0.1), spawn) == pytest.approx([-0.8 + math.cos(0.2), -0.8 + math.sin(0.2), 0.3])


def test_scan_arrays_follow_rep117_and_cut_off():
    ranges = [0.5, 1.2, float("inf"), float("nan"), float("-inf"), 0.05, 0.79]
    ang, r = scan_arrays(0.0, 0.1, ranges, range_min=0.12, range_max=3.5, cutoff=0.8)
    assert ang == pytest.approx(np.arange(7) * 0.1)
    assert r[0] == 0.5 and r[6] == 0.79                      # hits inside the cut-off
    assert np.isposinf(r[1]) and np.isposinf(r[2])            # beyond the cut-off / no return: free to 0.8
    assert np.isnan(r[3]) and np.isnan(r[4]) and np.isnan(r[5])   # invalid: skipped by integrate_scan


def test_scan_arrays_feed_the_grid():
    """nan readings must not clear cells (integrate_scan skips them)."""
    from nav_core.mapping import OCCUPIED, OccupancyGrid
    g = OccupancyGrid((-1, -1), (2, 2), 0.02)
    ang, r = scan_arrays(0.0, 0.1, [0.3], 0.12, 3.5, 0.8)
    g.integrate_scan((0.0, 0.0, 0.0), ang, r, 0.8, 0.12)
    ang, r = scan_arrays(0.0, 0.1, [float("nan")], 0.12, 3.5, 0.8)
    g.integrate_scan((0.0, 0.0, 0.0), ang, r, 0.8, 0.12)
    i, j, _ = g.world_to_cell([[0.3, 0.0]])
    assert g.state()[i[0], j[0]] == OCCUPIED


def test_occupancy_data_and_segments():
    st = np.array([[0, 1], [2, 1]])
    assert occupancy_data(st, unknown=0, free=1, occupied=2) == [-1, 0, 100, 0]
    assert segments_points([(np.array([0, 0]), np.array([1, 2]))]) == [(0.0, 0.0), (1.0, 2.0)]
