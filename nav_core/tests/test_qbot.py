"""The QBot (Dubins 4D): braking witnesses, the filters in model units, and closed loops in the arena"""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from nav_core.nominal import make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.robots.dynamics import Dubins4D
from nav_core.safety import make_filter
from nav_core.safety.hj_filter import hj_rates
from nav_core.safety.witness import find_witness_4d
from nav_core.sim import World
from nav_core.sim.loop import run_episode

PLANT = Dubins4D(-0.25, 1.0, 1.25, 1.0)          # the QBot in model units


def wall_scene(clearance=0.04, T=2.0):
    """The half plane y <= 0 as the modelled obstacle (margin = y), no primitives"""
    return SimpleNamespace(margin=lambda xy: np.atleast_2d(xy)[:, 1], clearance=clearance, T=T,
                           hj_control=lambda z: np.zeros((len(np.atleast_2d(z)), 2)))


def test_braking_witness_stops_before_the_wall():
    """Full speed straight at a wall: braking alone needs v^2 / (2 a) = 0.4; the witness stops exactly (v = 0) and
    keeps the clearance, and there is none when the wall is too close"""
    dt = 0.025
    found = find_witness_4d(np.array([0.0, 0.6, -math.pi / 2, 1.0]), wall_scene(), PLANT, dt, use_hj=False)
    assert found is not None
    plan = found[0]
    z = np.array([[0.0, 0.6, -math.pi / 2, 1.0]])
    low = np.inf
    for u in plan:
        z = PLANT.step_batch(z, u[None], dt)
        low = min(low, z[0, 1])
    assert z[0, 3] == pytest.approx(0.0, abs=1e-12) and low > 0.04
    assert find_witness_4d(np.array([0.0, 0.3, -math.pi / 2, 1.0]), wall_scene(), PLANT, dt, use_hj=False) is None
    # reversing towards the wall at v_min: brake with a > 0
    plan = find_witness_4d(np.array([0.0, 0.2, math.pi / 2, -0.25]), wall_scene(), PLANT, dt, use_hj=False)[0]
    assert plan[0, 0] > 0


def test_braking_witness_turns_away_when_braking_alone_is_not_enough():
    """At 45 degrees towards the wall, a bit closer than straight braking needs: turning away while braking helps"""
    x0 = np.array([0.0, 0.3, -math.pi / 4, 1.0])                  # braking straight: y drops by 0.4 sin 45 = 0.28
    found = find_witness_4d(x0, wall_scene(clearance=0.03), PLANT, 0.025, use_hj=False)
    assert found is not None and np.any(found[0][:, 1] != 0)


def test_hj_rates_4d_keep_the_nominal_or_evade():
    model = SimpleNamespace(hj_margin=lambda z: np.atleast_2d(z)[:, 1] - 0.5,
                            hj_control=lambda z: np.array([[-1.25, 1.0]]))
    far = np.array([0.0, 2.0, -math.pi / 2, 0.5])
    us, evaded = hj_rates(model, far, np.array([0.3, 0.0]), 2, 0.025, 0.0, PLANT)
    assert not evaded and us.shape == (2, 2) and np.allclose(us, [0.3, 0.0])
    near = np.array([0.0, 0.51, -math.pi / 2, 1.0])
    us, evaded = hj_rates(model, near, np.array([0.3, 0.0]), 2, 0.025, 0.0, PLANT)
    assert evaded and np.allclose(us, [[-1.25, 1.0]] * 2)


def test_qbot_filter_settings(v0_4d, v0):
    p = load_profile("qbot")
    assert not make_filter(p, "hj", v0=v0_4d).corners              # no 4D corner correction yet
    with pytest.raises(NotImplementedError, match="Dubins 3D"):
        make_filter(p, "hj", v0=v0_4d, corners=True)
    with pytest.raises(ValueError, match="Dubins 4D"):
        make_filter(p, "backup", v0=v0)                             # the Burger's 3D V0
    f = make_filter(p, "backup", v0=v0_4d)
    assert f.n_sub == 2 and not f.rect_margin
    assert f.to_model([1.6, -3.2, 0.5, 0.4]) == pytest.approx([1.0, -2.0, 0.5, 0.5])
    assert f.control_to_model([0.5, -0.5]) == pytest.approx([1.25, -1.0])
    assert np.allclose(f.controls_to_phys(np.array([[1.25, -1.0]])), [[0.5, -0.5]])


@pytest.mark.parametrize("kind", ["backup", "hj", "hj_backup"])
def test_qbot_reaches_the_goal_safely(v0_4d, kind):
    p = load_profile("qbot")
    world = World.from_sdf(ROBOTS_DIR.parent / p.world, plane_z=p.sensor.height)
    for x0, goal in (((-1.2, -1.2, 0.8), (1.2, 1.2)), ((1.2, 1.2, 3.1), (-1.2, -1.2))):
        f = make_filter(p, kind, v0=v0_4d)
        ep = run_episode(p, world, x0, goal, make_nominal(p, "mppi"), safety=f, t_max=30.0, goal_radius=0.15)
        assert ep.status == "reached", (kind, x0, ep.status, ep.error)
        assert world.distance(ep.traj[:, :2]).min() > p.footprint_radius
        assert ep.traj.shape[1] == 4 and ep.cmds.shape[1] == 4 and ep.traj[:, 3].max() > 0.5


def test_qbot_backup_filter_stops_pursuit_before_the_obstacle(v0_4d):
    """Pure pursuit drives into the L-shaped obstacle without a filter; with the backup filter it never collides"""
    p = load_profile("qbot")
    world = World.from_sdf(ROBOTS_DIR.parent / p.world, plane_z=p.sensor.height)
    x0, goal = (-1.2, 0.0, 0.0), (1.2, -1.2)
    assert run_episode(p, world, x0, goal, make_nominal(p, "pursuit"), t_max=10.0).status == "collision"
    f = make_filter(p, "backup", v0=v0_4d)
    ep = run_episode(p, world, x0, goal, make_nominal(p, "pursuit"), safety=f, t_max=10.0)
    assert ep.status in ("timeout", "reached") and world.distance(ep.traj[:, :2]).min() > p.footprint_radius
