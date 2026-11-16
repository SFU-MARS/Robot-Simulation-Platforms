import numpy as np
import pytest

from nav_core.mapping import OccupancyGrid, extract_obstacles
from nav_core.robots import load_profile
from nav_core.sim import Box, World

from test_nav_core import OBST_1, OBST_2, WALLS, ray_boxes
from test_safety import AROUND, PILLAR, scanned

POSES = [(x, y, 0.0) for x in (-0.6, 0.0, 0.6) for y in (-0.6, 0.85)] + [(-0.6, 0.3, 0.0), (0.6, 0.35, 0.0),
                                                                          (0.0, 0.0, 0.0)]
TRUTH = World([Box(((b[0] + b[1]) / 2, (b[2] + b[3]) / 2), ((b[1] - b[0]) / 2, (b[3] - b[2]) / 2))
               for b in (OBST_1, OBST_2, *WALLS)])
# outer corners of obstacles 1 and 2 (the others touch a wall or each other)
CONVEX = [(-0.1, 0.6), (0.3, 0.6), (0.3, 0.1), (-0.1, -0.3)]


def hj_map(noise=0.0, reps=1):
    p = load_profile("burger")
    g = OccupancyGrid.from_profile(p)
    rng = np.random.default_rng(0)
    angles = np.arange(360) * 2 * np.pi / 360
    for pose in POSES:
        for _ in range(reps):
            r = ray_boxes(np.array(pose[:2]), pose[2] + angles, (OBST_1, OBST_2, *WALLS))
            g.integrate_scan(pose, angles, r + (rng.normal(0, noise, r.shape) if noise else 0.0), 0.8)
    return g


def face_points(om):
    return np.vstack([f.a + np.linspace(0, 1, 50)[:, None] * (f.b - f.a) for f in om.faces])


def test_touching_obstacles_are_one():
    om = extract_obstacles(hj_map())
    assert om.n == 1                                           # walls, obstacle 1 and 2 touch (Design decision 2)
    assert len(om.faces) == 12


def test_convex_corners_are_the_outer_corners():
    om = extract_obstacles(hj_map())
    pts = np.array([c.point for c in om.corners if c.convex])
    assert len(pts) == len(CONVEX)
    for c in CONVEX:
        assert np.min(np.linalg.norm(pts - c, axis=1)) < 0.03
    for c in om.corners:                                       # a corner joins two faces of its obstacle
        i, j = c.faces
        assert om.faces[i].obstacle == om.faces[j].obstacle == c.obstacle
        assert np.allclose(om.faces[i].b, c.point) and np.allclose(om.faces[j].a, c.point)


def test_side_lengths_and_normals():
    om = extract_obstacles(hj_map())
    o1 = [f for f in om.faces if np.all([(-0.15 < q[0] < 0.35) and (0.05 < q[1] < 0.65) for q in (f.a, f.b)])]
    lengths = sorted(f.length for f in o1)
    assert lengths[-1] == pytest.approx(0.5, abs=0.04) and any(abs(L - 0.4) < 0.04 for L in lengths)
    for f in om.faces:                                         # the normal points into free space
        mid = 0.5 * (f.a + f.b)
        assert TRUTH.distance((mid + 0.03 * f.normal)[None])[0] > TRUTH.distance((mid - 0.03 * f.normal)[None])[0]


def test_faces_enclose_the_obstacles():
    """Noise-free: every face lies outside the true obstacles, i.e. encloses what was detected."""
    assert TRUTH.distance(face_points(extract_obstacles(hj_map()))).min() >= 0.0
    p, g = scanned(PILLAR, AROUND)
    assert PILLAR.distance(face_points(extract_obstacles(g))).min() >= 0.0


def test_pillar_and_offset():
    p, g = scanned(PILLAR, AROUND)
    om = extract_obstacles(g)
    assert om.n == 2                                           # the room and the free-standing pillar
    pillar = min(range(1, 3), key=lambda k: sum(f.length for f in om.faces_of(k)))
    L = [f.length for f in om.faces_of(pillar)]
    assert len(L) == 4 and all(0.2 <= x <= 0.26 for x in L)
    cs = [c for c in om.corners if c.obstacle == pillar]
    assert len(cs) == 4 and all(c.convex for c in cs)
    L2 = [f.length for f in extract_obstacles(g, offset=0.105).faces_of(pillar)]
    assert L2 == pytest.approx([x + 2 * 0.105 for x in L], abs=1e-6)      # configuration-space obstacle


def test_noise_does_not_fragment_walls():
    """1 cm range noise, 10 scans per place: about the noise-free face set."""
    om = extract_obstacles(hj_map(noise=0.01, reps=10))
    assert len(om.faces) <= 16
    assert 4 <= sum(c.convex for c in om.corners) <= 6
    assert TRUTH.distance(face_points(om)).min() > -0.01        # within the noise
