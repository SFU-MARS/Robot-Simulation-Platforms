import numpy as np
import pytest

from nav_core.mapping import OccupancyGrid, free_region
from nav_core.nominal import make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.sim import Box, World, run_episode

WORLD = ROBOTS_DIR.parent / "turtlebot3/turtlebot3_controls/hj_reach_target.world"


def scanned(world, poses):
    p = load_profile("burger")
    g = OccupancyGrid.from_profile(p)
    angles = np.arange(720) * 2 * np.pi / 720
    for x in poses:
        g.integrate_scan(x, angles, world.raycast(np.array(x[:2]), x[2] + angles, 2.0), 2.0)
    return p, g


ROOM = [Box((0.0, 1.0), (1.0, 0.02)), Box((0.0, -1.0), (1.0, 0.02)), Box((1.0, 0.0), (0.02, 1.0)), Box((-1.0, 0.0), (0.02, 1.0))]


def test_free_region_of_a_room():
    p, g = scanned(World(ROOM), [(0.0, 0.0, 0.0)])
    poly, holes = free_region(g, (0.0, 0.0), footprint_radius=0.1)
    assert holes == [] and len(poly) == 4                      # the room shrunk by the footprint, 4 corners
    assert np.abs(poly).max() == pytest.approx(0.98 - 0.1, abs=0.03)
    x, y = poly[:, 0], poly[:, 1]
    assert np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)) > 0          # counter-clockwise
    assert free_region(g, (0.95, 0.0), footprint_radius=0.1)[0] is None       # too close to a wall


PILLAR = World(ROOM + [Box((0.0, 0.0), (0.1, 0.1))])
AROUND = [(0.5, 0.5, 0.0), (-0.5, -0.5, 0.0), (0.5, -0.5, 0.0), (-0.5, 0.5, 0.0)]


def test_free_region_returns_free_standing_obstacles():
    p, g = scanned(PILLAR, AROUND)
    poly, holes = free_region(g, (0.5, 0.5), footprint_radius=0.05)
    assert len(holes) == 1                                     # the centre box, seen all around
    h = holes[0]
    assert np.abs(h).max() == pytest.approx(0.1 + 0.05, abs=0.03)          # box grown by the footprint


def test_backup_filter_avoids_the_collision(v0):
    from nav_core.safety import BackupFilter
    p = load_profile("burger")
    w = World.from_sdf(WORLD, plane_z=p.sensor.height)
    gate = BackupFilter(p, v0)
    ep = run_episode(p, w, (-0.8, -0.8, 0.2), (0.7, 0.2), make_nominal(p), safety=gate, t_max=30.0)
    assert ep.status == "reached"                              # nominal alone collides (test_sim)
    assert gate.interventions > 0
    assert w.distance(ep.traj[:, :2]).min() > p.footprint_radius


def test_backup_filter_loads_the_profile_v0(v0):
    from nav_core.safety import BackupFilter
    gate = BackupFilter(load_profile("burger"))                # robots/burger.yaml base_value
    assert gate.v0.encoding == "linear" and gate.alpha == 20.0 and gate.v0.meta["target_set_mode"] == "brt"
    assert gate.v0.meta["accuracy"] == "low"                   # the scheme of the local update


def test_backup_filter_rejects_a_missing_or_wrong_v0(tmp_path, fake_v0):
    import dataclasses
    from fast_brt.query import PrimitiveV0
    from nav_core.safety import BackupFilter
    p = load_profile("burger")
    with pytest.raises(FileNotFoundError, match="export-v0"):
        BackupFilter(p, tmp_path / "missing.npz")
    with pytest.raises(ValueError, match="base_value"):
        BackupFilter(dataclasses.replace(p, base_value=None))
    reach = PrimitiveV0(fake_v0.data, [-6.0, -6.0, -np.pi], [6.0, 6.0, np.pi], [241, 241, 60], [2], 20.0, "linear",
                        2.0, "dubins_3d", {"v": 1, "wMax": 1, "uMode": "min"})
    with pytest.raises(ValueError, match="avoid"):
        BackupFilter(p, reach)


@pytest.mark.parametrize("unknown", ["occupied", "free"])
def test_scene_covers_a_free_standing_obstacle(unknown, fake_v0):
    """Every robot position whose footprint touches the detected pillar is
    rejected: its margin is at most the clearance, and the filter only accepts
    margin > clearance."""
    from nav_core.safety import BackupFilter
    p, g = scanned(PILLAR, AROUND)
    gate = BackupFilter(p, fake_v0, unknown=unknown)
    scene = gate.scene(np.array([0.5, 0.5, 0.0]), g)
    xs = np.linspace(-0.1 - p.footprint_radius, 0.1 + p.footprint_radius, 41)
    pts = np.stack(np.meshgrid(xs, xs), -1).reshape(-1, 2)
    pts = pts[PILLAR.distance(pts) <= p.footprint_radius]      # footprint overlaps the pillar
    assert (scene.margin(gate.units.pos_to_model(pts)) <= scene.clearance).all()
    free_pt = gate.units.pos_to_model(np.array([[0.5, 0.5]]))
    assert scene.margin(free_pt)[0] > 0


@pytest.mark.parametrize("unknown", ["occupied", "free"])
def test_backup_filter_around_a_pillar(unknown, v0):
    from nav_core.safety import BackupFilter
    p = load_profile("burger")
    gate = BackupFilter(p, v0, unknown=unknown)
    ep = run_episode(p, PILLAR, (-0.6, -0.6, 0.0), (0.6, 0.6), make_nominal(p), safety=gate, t_max=60.0)
    assert ep.status in ("reached", "timeout"), ep.error
    assert PILLAR.distance(ep.traj[:, :2]).min() > p.footprint_radius


def test_place_faces_inward_and_centered(fake_v0):
    from nav_core.safety import place_faces
    a, b, n = np.array([0.0, 0.0]), np.array([4.0, 0.0]), np.array([0.0, 1.0])   # face on y = 0, free above
    (c, hl, hw, beta), = place_faces(fake_v0, [(a, b, n)], "inward").rectangles()
    assert hl == pytest.approx(2.0) and hw == pytest.approx(2.0 / np.sqrt(20.0)) and beta == pytest.approx(0.0)
    assert c == pytest.approx([2.0, -hw])                       # rectangle y in [-2 hw, 0]: thickness behind the face
    (c2, _, hw2, _), = place_faces(fake_v0, [(a, b, n)], "centered").rectangles()
    assert c2 == pytest.approx([2.0, 0.0]) and hw2 == pytest.approx(hw)    # the face as midline
    pieces = place_faces(fake_v0, [(a, b, n)], "inward", max_side_len=1.0).rectangles()
    assert len(pieces) == 4 and all(p[1] == pytest.approx(0.5) for p in pieces)
    assert len(place_faces(fake_v0, [], "inward")) == 0
    with pytest.raises(ValueError):
        place_faces(fake_v0, [(a, b, n)], "sideways")


def test_scene_value_and_hj_control(fake_v0):
    from nav_core.safety import Scene, place_faces
    faces = place_faces(fake_v0, [(np.array([-1.0, 0.0]), np.array([1.0, 0.0]), np.array([0.0, 1.0]))])
    scene = Scene(fake_v0, faces, None, clearance=0.3, horizon=2.0)
    states = np.array([[0.0, 0.5, 0.0], [0.0, -2.0, 1.0]])
    assert scene.value(states) == pytest.approx(fake_v0.value(states, faces))
    assert scene.margin(states[:, :2]) == pytest.approx([0.5, 2.0 - 2 * faces.half_wid[0]])
    assert np.all(np.abs(scene.hj_control(states)) == 1.0)
    empty = Scene(fake_v0, place_faces(fake_v0, []), None, clearance=0.3, horizon=2.0)
    assert np.all(np.isinf(empty.value(states))) and np.all(empty.hj_control(states) == 0.0)
    assert np.all(np.isinf(empty.margin(states)))


def test_inward_rectangles_stay_behind_the_faces(fake_v0):
    """Inward placement on a scanned map: every rectangle's back edge is on the obstacle
    side of the region boundary, and its front edge reaches at most two cells into the
    free region (faces are enclosing, so they sit slightly outside the region contour)."""
    from nav_core.safety import BackupFilter
    from nav_core.safety.backup_filter import _contains
    p, g = scanned(PILLAR, AROUND)
    gate = BackupFilter(p, fake_v0)
    scene = gate.scene(np.array([0.5, 0.5, 0.0]), g)
    cell = p.map.resolution / p.units.length

    def region_sd(pts):                         # signed distance to the region alone (+ inside)
        inside = _contains(scene.rings[0], pts)
        for hole in scene.rings[1:]:
            inside &= ~_contains(hole, pts)
        d = scene._seg_dist(pts, scene._RA, scene._RB)
        return np.where(inside, d, -d)

    assert scene.frames
    for c, hl, hw, beta in scene.frames:
        nrm = np.array([-np.sin(beta), np.cos(beta)])
        front, back = sorted(region_sd(np.array([np.asarray(c) + hw * nrm, np.asarray(c) - hw * nrm])), reverse=True)
        assert front <= 2 * cell
        assert back <= 0.0
