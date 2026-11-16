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


def test_gatekeeper_avoids_the_collision():
    pytest.importorskip("fast_brt")
    from nav_core.safety import FastBrtGatekeeper
    p = load_profile("burger")
    w = World.from_sdf(WORLD, plane_z=p.sensor.height)
    gate = FastBrtGatekeeper(p)
    ep = run_episode(p, w, (-0.8, -0.8, 0.2), (0.7, 0.2), make_nominal(p), safety=gate, t_max=30.0)
    assert ep.status == "reached"                              # nominal alone collides (test_sim)
    assert gate.interventions > 0
    assert w.distance(ep.traj[:, :2]).min() > p.footprint_radius


@pytest.mark.parametrize("unknown", ["occupied", "free"])
def test_scene_covers_a_free_standing_obstacle(unknown):
    """Every robot position whose footprint touches the detected pillar is
    rejected: its margin is at most the clearance, and the filter only accepts
    margin > clearance."""
    pytest.importorskip("fast_brt")
    from nav_core.safety import FastBrtGatekeeper
    p, g = scanned(PILLAR, AROUND)
    gate = FastBrtGatekeeper(p, unknown=unknown)
    scene = gate.scene(np.array([0.5, 0.5, 0.0]), g)
    xs = np.linspace(-0.1 - p.footprint_radius, 0.1 + p.footprint_radius, 41)
    pts = np.stack(np.meshgrid(xs, xs), -1).reshape(-1, 2)
    pts = pts[PILLAR.distance(pts) <= p.footprint_radius]      # footprint overlaps the pillar
    assert (scene.margin(gate.units.pos_to_model(pts)) <= scene.clearance).all()
    free_pt = gate.units.pos_to_model(np.array([[0.5, 0.5]]))
    assert scene.margin(free_pt)[0] > 0


@pytest.mark.parametrize("unknown", ["occupied", "free"])
def test_gatekeeper_around_a_pillar(unknown):
    pytest.importorskip("fast_brt")
    from nav_core.safety import FastBrtGatekeeper
    p = load_profile("burger")
    gate = FastBrtGatekeeper(p, unknown=unknown)
    ep = run_episode(p, PILLAR, (-0.6, -0.6, 0.0), (0.6, 0.6), make_nominal(p), safety=gate, t_max=60.0)
    assert ep.status in ("reached", "timeout"), ep.error
    assert PILLAR.distance(ep.traj[:, :2]).min() > p.footprint_radius
