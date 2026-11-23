import dataclasses

import numpy as np
import pytest

from nav_core.mapping import OccupancyGrid
from nav_core.nominal import MPPI, CostMap, make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.sim import World, run_episode

HJ = ROBOTS_DIR.parent / "turtlebot3/turtlebot3_controls/hj_reach_target.world"
OCCLUDED = ROBOTS_DIR.parent / "turtlebot3/turtlebot3_controls/worlds/occluded_4x4.world"


def test_cost_to_go_is_octile_in_free_space_and_goes_around_walls():
    g = OccupancyGrid((-1, -1), (2, 2), 0.02)
    cm = CostMap(g, (0.0, 0.0), 0.1)
    for q in [(0.5, 0.0), (0.5, 0.5), (-0.3, 0.7)]:
        dx, dy = abs(q[0]), abs(q[1])
        assert cm.lookup(np.array([q]))[0][0] == pytest.approx(max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy), abs=0.03)
    g = OccupancyGrid((-1, -1), (2, 2), 0.02, model="logodds")
    g.logodds[:int(1.5 / 0.02), int(1.3 / 0.02)] = 4.0           # wall x = 0.3, y in [-1, 0.5]
    cm = CostMap(g, (0.6, 0.0), 0.1)
    assert cm.lookup(np.array([[0.0, 0.0]]))[0][0] > 1.2           # around the wall's end, not 0.6 straight


def test_mppi_reaches_the_goal_where_pursuit_collides():
    p = load_profile("burger")
    w = World.from_sdf(HJ, plane_z=p.sensor.height)
    assert run_episode(p, w, (-0.8, -0.8, 0.2), (0.7, 0.2), make_nominal(p, "pursuit"), t_max=40).status == "collision"
    ep = run_episode(p, w, (-0.8, -0.8, 0.2), (0.7, 0.2), make_nominal(p, "mppi"), t_max=40)
    assert ep.status == "reached"
    assert isinstance(make_nominal(p, "mppi"), MPPI)


def test_occluded_world():
    p = load_profile("burger")
    w = World.from_sdf(OCCLUDED, plane_z=p.sensor.height)
    assert len(w.boxes) == 13                                       # 4 walls, 3 screens, 3 hidden boxes, 3 pillars
    origin, size = w.bounds()
    assert origin == pytest.approx((-2.1, -2.1)) and size == pytest.approx((4.2, 4.2))


def test_short_sensing_range_cannot_certify_the_start(v0):
    """unknown=occupied needs the first scan to hold a loiter circle: range >~ footprint + 2 turning radii."""
    from nav_core.safety import BackupFilter
    p0 = load_profile("burger")
    w = World.from_sdf(OCCLUDED, plane_z=p0.sensor.height)
    o, s = w.bounds()
    for R, expect in ((0.3, "init_failed"), (0.8, "reached")):
        p = dataclasses.replace(p0, sensor=dataclasses.replace(p0.sensor, range=R))
        ep = run_episode(p, w, (-0.9, -1.4, 0.0), (0.9, -0.3), make_nominal(p, "mppi"),
                         safety=BackupFilter(p, v0), t_max=60, grid=OccupancyGrid(o, s, p.map.resolution))
        assert ep.status == expect
