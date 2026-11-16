import numpy as np
import pytest

from nav_core.mapping import OCCUPIED, OccupancyGrid, boundary_edges
from nav_core.nominal import make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.sim import Box, World, run_episode, unicycle_step

WORLD = ROBOTS_DIR.parent / "turtlebot3/turtlebot3_controls/hj_reach_target.world"


def burger_world():
    p = load_profile("burger")
    return p, World.from_sdf(WORLD, plane_z=p.sensor.height)


def test_sdf_boxes():
    _, w = burger_world()
    assert len(w.boxes) == 6                                   # 4 walls + 2 obstacles; floor and markers dropped
    b = min(w.boxes, key=lambda b: np.hypot(b.center[0] - 0.1, b.center[1] - 0.35))
    assert b.center == pytest.approx((0.1, 0.35)) and b.half == pytest.approx((0.2, 0.25))


def test_raycast():
    _, w = burger_world()
    r = w.raycast(np.array([-0.8, -0.8]), np.array([np.pi / 2, 0.0, np.pi]), range_max=0.8)
    assert r[0] == pytest.approx(0.5)                          # bottom face of obstacle 2 at y = -0.3
    assert np.isinf(r[1])                                      # east wall beyond range
    assert r[2] == pytest.approx(0.2)                          # west wall at x = -1
    diamond = World([Box((1.0, 0.0), (0.5, 0.5), np.pi / 4)])
    assert diamond.raycast(np.zeros(2), np.array([0.0]), 5.0)[0] == pytest.approx(1.0 - 0.5 * np.sqrt(2))
    assert diamond.distance(np.array([[0.0, 0.0]]))[0] == pytest.approx(1.0 - 0.5 * np.sqrt(2))


def test_invalid_readings_do_not_clear_cells():
    g = OccupancyGrid.from_profile(load_profile("burger"))
    g.integrate_scan((0.0, 0.0, 0.0), [0.0], [0.3], 0.8)
    for bad in (np.nan, -np.inf, 0.05):
        g.integrate_scan((0.0, 0.0, 0.0), [0.0], [bad], 0.8, range_min=0.12)
    i, j, _ = g.world_to_cell([[0.3, 0.0]])
    assert g.state()[i[0], j[0]] == OCCUPIED


def test_unicycle_step_is_exact_arc():
    x = np.zeros(3)
    for _ in range(100):
        x = unicycle_step(x, 1.0, 1.0, np.pi / 100)            # half a unit circle
    assert x[:2] == pytest.approx([0.0, 2.0], abs=1e-9)


def test_free_space_after_one_scan():
    p, w = burger_world()
    g = OccupancyGrid.from_profile(p)
    angles = np.arange(p.sensor.n_rays) * p.sensor.fov / p.sensor.n_rays
    x = np.array([-0.8, -0.8, 0.2])
    g.integrate_scan(x, angles, w.raycast(x[:2], x[2] + angles, p.sensor.range), p.sensor.range, p.sensor.range_min)
    assert boundary_edges(g, footprint_radius=p.footprint_radius)   # the filter has free space at t = 0


def test_empty_world_reaches_goal():
    p = load_profile("burger")
    ep = run_episode(p, World([]), (-0.8, -0.8, 0.2), (0.7, 0.2), make_nominal(p), t_max=30.0)
    assert ep.status == "reached"


def test_nominal_only_collides_in_hj_world():
    """Pure pursuit grazes obstacle 2's corner: what the safety filter is for."""
    p, w = burger_world()
    ep = run_episode(p, w, (-0.8, -0.8, 0.2), (0.7, 0.2), make_nominal(p), t_max=30.0)
    assert ep.status == "collision"
    assert w.distance(ep.traj[-1:, :2])[0] <= p.footprint_radius


def test_safety_hook_overrides_nominal():
    p, w = burger_world()
    ep = run_episode(p, World([]), (0.0, 0.0, 0.0), (0.9, 0.9), make_nominal(p),
                     safety=lambda x, w_nom, grid: 5.0, t_max=4.0)
    assert ep.status == "timeout"
    assert np.all(ep.cmds[:, 1] == p.phys["w_max"])            # hook output, clipped to w_max
