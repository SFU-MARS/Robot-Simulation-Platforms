import numpy as np
import pytest

from nav_core.mapping import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, boundary_edges, simplify_polyline
from nav_core.nominal import Pursuit, make_nominal
from nav_core.robots import load_profile

# hj_reach_target.world: (xmin, xmax, ymin, ymax) of the two obstacles and the inner wall faces
OBST_1 = (-0.1, 0.3, 0.1, 0.6)
OBST_2 = (-1.0, -0.1, -0.3, 0.1)
WALLS = [(-1.05, 1.05, 1.0, 1.05), (-1.05, 1.05, -1.05, -1.0), (-1.05, -1.0, -1.05, 1.05), (1.0, 1.05, -1.05, 1.05)]


def ray_boxes(pos, phi, boxes):
    """Distance along each ray to the nearest axis-aligned box (slab method)."""
    d = np.stack([np.cos(phi), np.sin(phi)], 1)
    inv = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
    r = np.full(len(phi), np.inf)
    for x0, x1, y0, y1 in boxes:
        t1 = (np.array([x0, y0]) - pos) * inv
        t2 = (np.array([x1, y1]) - pos) * inv
        tn, tf = np.minimum(t1, t2).max(1), np.maximum(t1, t2).min(1)
        ok = (tn <= tf) & (tn > 0)
        r = np.where(ok, np.minimum(r, tn), r)
    return r


def scan(grid, pose, rng=0.8, n=360, boxes=(OBST_1, OBST_2, *WALLS)):
    angles = np.arange(n) * 2 * np.pi / n
    grid.integrate_scan(pose, angles, ray_boxes(np.array(pose[:2]), pose[2] + angles, boxes), rng)


def test_profiles():
    p = load_profile("burger").require_ready()
    assert p.units.length == pytest.approx(0.2) and p.units.time == pytest.approx(1.0)
    assert p.units.pos_to_model([0.2, 0.4]) == pytest.approx([1.0, 2.0])
    for name in ("qbot", "crazyflie"):
        with pytest.raises(RuntimeError):
            load_profile(name).require_ready()


def test_scan_marks_free_hit_and_shadow():
    g = OccupancyGrid.from_profile(load_profile("burger"))
    for _ in range(2):                                    # two scans pass the 0.6 log-odds threshold
        scan(g, (-0.8, -0.8, 0.0))
    s = g.state()
    cell = lambda x, y: s[g.world_to_cell([[x, y]])[0][0], g.world_to_cell([[x, y]])[1][0]]
    assert cell(-0.5, -0.8) == FREE
    assert cell(-0.5, -0.3) == OCCUPIED                   # hit on the bottom face of obstacle 2
    assert cell(-0.5, 0.5) == UNKNOWN                     # behind obstacle 2
    assert cell(0.9, 0.9) == UNKNOWN                      # out of range


def test_accumulated_map_recovers_obstacle_sides():
    g = OccupancyGrid.from_profile(load_profile("burger"))
    poses = [(x, y, 0.0) for x in (-0.6, 0.0, 0.6) for y in (-0.6, 0.85)] + [(-0.6, 0.3, 0.0), (0.6, 0.35, 0.0), (0.0, 0.0, 0.0)]
    for pose in poses:
        for _ in range(2):
            scan(g, pose)
    edges = boundary_edges(g)
    obst = [e for e in edges if e.kind == "obstacle"]
    assert obst
    near = lambda e, box: all(box[0] - 0.05 <= p[0] <= box[1] + 0.05 and box[2] - 0.05 <= p[1] <= box[3] + 0.05
                              for p in (e.a, e.b))
    o1 = sorted(e.length for e in obst if near(e, OBST_1))
    # the four sides of obstacle 1 (0.4 x 0.5) are seen; each estimate is within two cells
    assert len(o1) >= 4
    assert o1[-1] == pytest.approx(0.5, abs=0.04)
    assert any(abs(L - 0.4) < 0.04 for L in o1)


def test_footprint_shrinks_free_space():
    g = OccupancyGrid.from_profile(load_profile("burger"))
    for _ in range(2):
        scan(g, (-0.5, -0.65, 0.0))
    bottom = lambda edges: min(min(e.a[1], e.b[1]) for e in edges if e.kind == "obstacle")
    y0 = bottom(boundary_edges(g))
    y1 = bottom(boundary_edges(g, footprint_radius=0.105))
    assert y1 - y0 == pytest.approx(0.105, abs=0.03)       # bottom wall edge moves up by the footprint


def test_frontier_at_range_limit():
    g = OccupancyGrid.from_profile(load_profile("burger"))
    for _ in range(2):
        scan(g, (0.4, -0.5, 0.0), rng=0.3, boxes=())
    edges = boundary_edges(g)
    assert edges and all(e.kind == "frontier" for e in edges)
    assert sum(e.length for e in edges) == pytest.approx(2 * np.pi * 0.3, rel=0.15)


def test_simplify_polyline_keeps_corner():
    pts = np.array([[0, 0], [1, 0.001], [2, 0], [2, 1], [2, 2]], float)
    assert simplify_polyline(pts, 0.01).tolist() == [[0, 0], [2, 0], [2, 2]]


def test_pursuit():
    nom = make_nominal(load_profile("burger"))
    assert isinstance(nom, Pursuit)
    assert nom.control((0, 0, 0), (1, 0)) == pytest.approx(0.0)
    assert nom.control((0, 0, 0), (0, 1)) == pytest.approx(1.0)       # saturated at w_max
    assert nom.control((0, 0, 0), (0, -1)) == pytest.approx(-1.0)
