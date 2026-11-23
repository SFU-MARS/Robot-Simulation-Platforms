import numpy as np
import pytest

from nav_core.safety.witness import N_CIRCLE, arc_step, candidate_controls, find_witness, loiter_sides, witness_slack
from nav_core.sim.loop import unicycle_step


class Disc:
    """Free space = a disc of radius r around the origin (model units)."""

    def __init__(self, r, clearance=0.3, T=2.0, w_hj=0.0):
        self.r, self.clearance, self.T, self.w_hj = r, clearance, T, w_hj

    def margin(self, xy):
        return self.r - np.linalg.norm(np.atleast_2d(xy)[:, :2], axis=1)

    def hj_control(self, states):
        return np.full(len(np.atleast_2d(states)), self.w_hj)


def test_arc_step_is_the_exact_unicycle_step():
    z = np.array([[0.1, -0.2, 0.3], [1.0, 2.0, -2.0], [0.0, 0.0, 3.0]])
    w = np.array([1.0, -0.4, 0.0])
    expected = np.array([unicycle_step(x, 1.0, wi, 0.37) for x, wi in zip(z, w)])
    arc_step(z, w, 0.37)
    assert z[:, :2] == pytest.approx(expected[:, :2])
    assert np.cos(z[:, 2]) == pytest.approx(np.cos(expected[:, 2]))


def test_candidates_and_slack():
    W = candidate_controls(40)
    assert W.shape == (3 + 2 * 7, 40)                           # L, R, S, then k = 5, ..., 35 for both turns
    assert np.all(W[3, :5] == 0) and np.all(W[3, 5:] == 1.0)
    assert witness_slack(0.05) == pytest.approx(2 * np.sin(np.pi / (2 * N_CIRCLE)))


def test_witness_in_a_large_disc_and_none_in_a_small_one():
    found = find_witness(np.array([0.0, 0.0, 0.0]), Disc(3.0, w_hj=1.0), 0.05)
    assert found is not None
    controls, side = found
    assert np.all(controls == 1.0) and side == 1.0             # the HJ candidate comes first and loiters left
    assert find_witness(np.array([0.0, 0.0, 0.0]), Disc(1.2), 0.05) is None     # no free circle of radius 1
    with pytest.raises(ValueError, match="slack"):
        find_witness(np.array([0.0, 0.0, 0.0]), Disc(3.0, clearance=0.01), 0.05)


def test_loiter_sides():
    scene = Disc(2.5)
    # heading +x at (0, 1): the left circle (centre (0, 2)) leaves the disc, the right one (centre (0, 0)) is free
    assert list(loiter_sides(np.array([[0.0, 1.0, 0.0], [0.0, -1.0, 0.0], [0.0, 2.4, 0.0]]), scene)) == [-1.0, 1.0, 0.0]
