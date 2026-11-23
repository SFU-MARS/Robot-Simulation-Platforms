"""
What the safety filters see each control period, in model units (v = 1, |w| <= 1; ``profile.units`` converts).

The accumulated map, shrunk by the robot footprint (so the robot is a point), gives the region the robot may use
and the edges the filter must respect; every edge becomes one primitive whose HJ value comes from an offline V0
(``python -m fast_brt export-v0`` in fast-brt), read by ``fast_brt.query.PrimitiveV0`` (numpy only).  ``v0`` is a
``PrimitiveV0``, a V0 file, or None for the profile's ``base_value`` (a path relative to the repository root).
V0 also sets alpha.

``unknown`` chooses what the filter must respect:

  "occupied" -- unknown space is an obstacle.  The region is the known free
      space around the robot; frontier edges are obstacle edges too.  Safe
      against obstacles hidden behind others, more conservative.
  "free" -- only detected obstacles count; unknown space is free.  The region
      is everything but the detected obstacles, each enclosed by its own ring.
      Less conservative, but safe only if every obstacle is seen before the
      robot enters its BRT (sensing range, no occlusion surprise).

``primitives`` chooses where the primitives come from:

  "faces" -- the obstacle layer: enclosing faces of every obstacle, shifted by
      the footprint, plus the frontier edges in the "occupied" mode.  The
      obstacle map of the last period (``self.obstacles``, stable ids) also
      holds the corner list; ``self.events`` lists what changed in that
      period (``ObstacleTracker``), e.g. for a leaking-corner update.
  "rings" -- every edge of the region's rings.

``placement`` puts each primitive rectangle (length L, thickness L/sqrt(alpha))
on its edge:

  "inward" -- the rectangle's free-facing side lies on the edge and its
      thickness is inside the obstacle, so how far the primitive (and its BRT)
      reaches into free space depends on L only; long edges need no cutting.
  "centered" -- the edge is the rectangle's midline (the earlier behaviour):
      half the thickness reaches into free space, which ``cut`` limits by
      cutting edges into pieces of at most 2 h_max sqrt(alpha).

``clearance`` (m) covers what the grid cannot resolve: the marching-squares
contour and the cell-centre distances can place an edge up to about one cell
beyond the truly free space.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

from ..mapping import ObstacleTracker, boundary_edges, extract_obstacles, free_region, safe_free_mask

__all__ = ["REPO_ROOT", "V0_ENV", "Scene", "SceneFilter", "check_v0", "import_query", "load_v0", "place_faces"]

REPO_ROOT = Path(__file__).resolve().parents[3]
V0_ENV = "ROBOT_SIM_V0"                       # environment variable: a V0 file used instead of the profile's


def import_query():
    """The module ``fast_brt.query`` (numpy only), from the environment, else from the fast-brt submodule of
    this repository (added to sys.path)."""
    try:
        import fast_brt.query as query
    except ImportError:
        submodule = REPO_ROOT / "fast-brt"
        if "fast_brt" in sys.modules or not (submodule / "fast_brt" / "query.py").is_file():
            raise ImportError("fast_brt.query is not importable: put SFU-MARS/fast-brt (branch robot-sim) on "
                              "PYTHONPATH, not Hu-Hanyang/FaST-BRT, or check out the fast-brt submodule") from None
        sys.path.insert(0, str(submodule))
        import fast_brt.query as query
    return query


def load_v0(v0, root=None):
    """A ``fast_brt.query.PrimitiveV0`` from a V0 file, or ``v0`` itself if it is one.  Relative paths start at
    ``root`` (default: this repository)."""
    PrimitiveV0 = import_query().PrimitiveV0
    if isinstance(v0, PrimitiveV0):
        return v0
    path = Path(v0).expanduser()
    path = path if path.is_absolute() else Path(root or REPO_ROOT) / path
    if not path.is_file():
        raise FileNotFoundError(f"no V0 file at {path}: download it with `python scripts/fetch_v0.py` in nav_core, or "
                                "export it with `python -m fast_brt export-v0` in fast-brt (odp environment)")
    check_v0(path)
    return PrimitiveV0.load(path)


def check_v0(path, manifest=None) -> None:
    """If robots/v0_release.yaml lists a file of this name, its sha256 must match: a V0 that was re-exported or
    replaced without updating the release fails loudly instead of being used silently."""
    import hashlib
    import yaml
    manifest = Path(manifest) if manifest is not None else REPO_ROOT / "robots" / "v0_release.yaml"
    if not manifest.is_file():
        return
    expected = (yaml.safe_load(manifest.read_text()) or {}).get("files", {}).get(Path(path).name)
    if expected is None:
        return
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    if h.hexdigest() != expected:
        raise ValueError(f"{path} does not match the sha256 in {manifest}: run `python scripts/fetch_v0.py`, or "
                         "publish the new V0 under a new release tag and update the manifest")


def _left_normal(a, b) -> np.ndarray:
    d = np.subtract(b, a)
    return np.array([-d[1], d[0]]) / max(float(np.linalg.norm(d)), 1e-12)


def _free_normal(grid, safe: np.ndarray, a, b) -> np.ndarray:
    """Unit normal of segment a -> b (metres) pointing to the side where ``safe`` (a cell mask) is free."""
    n = _left_normal(a, b)
    mid = 0.5 * (np.asarray(a, float) + np.asarray(b, float))
    i, j, inside = grid.world_to_cell(np.stack([mid + 0.75 * grid.res * n, mid - 0.75 * grid.res * n]))
    free = [bool(ok and safe[ii, jj]) for ii, jj, ok in zip(i, j, inside)]
    return n if free[0] or not free[1] else -n


def _contains(ring: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Even-odd point-in-polygon for many points."""
    x, y = ring[:, 0], ring[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    px, py = xy[:, :1], xy[:, 1:2]
    dy = np.where(y2 == y, 1e-300, y2 - y)
    crosses = ((y > py) != (y2 > py)) & (px < x + (py - y) * (x2 - x) / dy)
    return np.count_nonzero(crosses, axis=1) % 2 == 1


def place_faces(v0, segments, placement: str = "inward", max_side_len: "float | None" = None):
    """``fast_brt.query.Faces`` for segments (a, b, n), n the unit normal of the segment pointing into
    free space.  ``placement``: "inward" or "centered" (see the module docstring); ``max_side_len``
    (None: off) first cuts each segment into equal collinear pieces of at most that length.  Short
    faces are extended and long ones capped by ``v0.place``."""
    if placement not in ("inward", "centered"):
        raise ValueError(f"placement must be 'inward' or 'centered', got {placement!r}")
    a, b, n = [], [], []
    for pa, pb, pn in segments:
        pa, pb = np.asarray(pa, float), np.asarray(pb, float)
        length = float(np.linalg.norm(pb - pa))
        if length <= 1e-9:
            continue
        k = 1 if max_side_len is None else max(1, int(np.ceil(length / max_side_len - 1e-9)))
        ts = np.linspace(0.0, 1.0, k + 1)
        for t0, t1 in zip(ts[:-1], ts[1:]):
            a.append(pa + t0 * (pb - pa))
            b.append(pa + t1 * (pb - pa))
            n.append(np.asarray(pn, float))
    a, b, n = (np.reshape(np.array(v, float), (-1, 2)) for v in (a, b, n))
    faces = v0.place(a, b, n)
    if placement == "centered":                 # move each face half a thickness into free space
        shift = np.stack([-np.sin(faces.beta), np.cos(faces.beta)], axis=1) * faces.half_wid[:, None]
        faces = v0.place(a + shift, b + shift, n)
    return faces


class Scene:
    """What the backup filter sees in one period, in model units.

    ``rings``: [outer, hole, ...] polygons of the free region, or None (no
    region: unknown space is free).  ``faces``: the primitives
    (``fast_brt.query.Faces``); ``frames`` are their rectangles (center, half
    length, half width, angle).  Modelled obstacle = (complement of the
    region, if any) U (primitive rectangles); ``margin`` is the signed
    distance to it (positive = free)."""

    def __init__(self, v0, faces, rings, clearance: float, horizon: float):
        self.v0, self.faces = v0, faces
        self.frames = faces.rectangles()
        self.rings = rings
        self._C = faces.center
        self._HL, self._HW = faces.half_len, faces.half_wid
        self._cos, self._sin = np.cos(faces.beta), np.sin(faces.beta)
        if rings:
            ring_edges = [(a, b) for r in rings for a, b in zip(r, np.roll(r, -1, axis=0))]
            self._RA = np.array([e[0] for e in ring_edges])
            self._RB = np.array([e[1] for e in ring_edges])
        self.clearance = clearance
        self.T = horizon

    @staticmethod
    def _seg_dist(xy, A, B):
        ab = B - A
        ap = xy[:, None, :] - A[None]
        t = np.clip(np.sum(ap * ab[None], -1) / np.maximum(np.sum(ab * ab, -1), 1e-12)[None], 0, 1)
        return np.linalg.norm(ap - t[..., None] * ab[None], axis=-1).min(axis=1)

    def margin(self, xy: np.ndarray) -> np.ndarray:
        xy = np.atleast_2d(xy)[:, :2]
        m = np.full(len(xy), np.inf)
        if self.rings:
            inside = _contains(self.rings[0], xy)
            for hole in self.rings[1:]:
                inside &= ~_contains(hole, xy)
            d = self._seg_dist(xy, self._RA, self._RB)
            m = np.where(inside, d, -d)
        if len(self.faces):
            d = xy[None] - self._C[:, None]
            xl = self._cos[:, None] * d[..., 0] + self._sin[:, None] * d[..., 1]
            yl = -self._sin[:, None] * d[..., 0] + self._cos[:, None] * d[..., 1]
            rect = np.maximum(np.abs(xl) - self._HL[:, None], np.abs(yl) - self._HW[:, None]).min(axis=0)
            m = np.minimum(m, rect)
        return m

    def value(self, states: np.ndarray) -> np.ndarray:
        """HJ value of the union of the primitives at states (N, 3); inf without primitives."""
        return self.v0.value(states, self.faces)

    def hj_margin(self, states: np.ndarray) -> np.ndarray:
        """Distance margin of the union (model length units, ``PrimitiveV0.margin``): every state within it (same
        heading) is outside the BRT of every primitive; inf without primitives."""
        return self.v0.margin(states, self.faces)

    def hj_control(self, states: np.ndarray) -> np.ndarray:
        """Optimal turn rate of the nearest primitive, w = sign(dV/dtheta); straight without primitives."""
        states = np.atleast_2d(states)
        if not len(self.faces):
            return np.zeros(len(states))
        return self.v0.query(states, self.faces).control[:, 0]


class SceneFilter:
    """Common part of the filters: the profile, V0, units, and the scene of each period (``scene``).

    V0: ``v0`` if given, else the file in the environment variable ROBOT_SIM_V0, else the profile's
    ``base_value``; relative paths start at the folder that holds the profile's robots/ (``profile.root``)."""

    uses_region = True                         # filters that only use the primitives set False: no region is built

    def __init__(self, profile, v0=None, unknown: str = "occupied", primitives: str = "faces",
                 placement: str = "inward", cut: bool = False, horizon: float = 2.0, sub_dt: float = 0.05,
                 clearance: "float | None" = None, h_max: "float | None" = None):
        profile.require_ready()
        if profile.dynamics != "dubins3d":
            raise NotImplementedError(f"{type(self).__name__} for dynamics {profile.dynamics!r}")
        if unknown not in ("occupied", "free"):
            raise ValueError(f"unknown must be 'occupied' or 'free', got {unknown!r}")
        if primitives not in ("faces", "rings"):
            raise ValueError(f"primitives must be 'faces' or 'rings', got {primitives!r}")
        if placement not in ("inward", "centered"):
            raise ValueError(f"placement must be 'inward' or 'centered', got {placement!r}")
        if v0 is None:
            v0 = os.environ.get(V0_ENV) or profile.base_value
            if v0 is None:
                raise ValueError(f"no V0: set base_value in robots/{profile.name}.yaml, {V0_ENV}, or pass v0")
        self.v0 = load_v0(v0, getattr(profile, "root", None))
        p = self.v0.dynamics_params
        if (self.v0.dynamics != "dubins_3d" or p.get("v", 1) != 1 or p.get("wMax", 1) != 1
                or p.get("uMode", "max") != "max"):
            raise ValueError(f"V0 must be the avoid value of the model Dubins 3D (v = wMax = 1, uMode max), got "
                             f"{self.v0.dynamics} {p}")
        self.alpha = self.v0.alpha
        self.profile, self.units, self.unknown, self.primitives = profile, profile.units, unknown, primitives
        self.placement = placement
        self.obstacles = None                          # ObstacleMap of the last period ("faces"), stable ids
        self.events = []                               # ObstacleEvents of the last period
        self.tracker = ObstacleTracker()
        res = profile.map.resolution
        self.clearance = (1.5 * res if clearance is None else clearance) / self.units.length
        h = (res if h_max is None else h_max) / self.units.length
        self.max_side_len = 2.0 * h * np.sqrt(self.alpha) if cut else None
        ctrl_dt = profile.ctrl_dt / self.units.time
        self.horizon, self.sub_dt = horizon, sub_dt
        self.n_sub = int(round(ctrl_dt / sub_dt))
        if not np.isclose(self.n_sub * sub_dt, ctrl_dt):
            raise ValueError(f"ctrl_dt {ctrl_dt} (model time) is not a multiple of sub_dt {sub_dt}")
        self.steps = self.interventions = 0
        self.model = None                              # last scene, for plots

    def to_model(self, x) -> np.ndarray:
        return np.array([*self.units.pos_to_model(x[:2]), x[2] % (2 * np.pi)])

    def scene(self, x, grid) -> Scene:
        """The scene (model units) the filter sees at pose ``x``."""
        to_m = self.units.pos_to_model
        r = self.profile.footprint_radius
        rings = None
        if self.uses_region or self.primitives == "rings":
            outer, holes = free_region(grid, x[:2], footprint_radius=r, unknown_free=(self.unknown == "free"))
            if outer is None:
                raise RuntimeError("robot is outside the free region")
            rings = [to_m(outer)] + [to_m(h) for h in holes]
        if self.primitives == "rings":
            # outer ring counter-clockwise, holes clockwise: the free region is on the left of every edge
            segments = [(a, b, _left_normal(a, b)) for ring in rings for a, b in zip(ring, np.roll(ring, -1, axis=0))]
        else:
            self.obstacles, self.events = self.tracker.update(extract_obstacles(grid, offset=r), res=grid.res)
            segments = [(to_m(f.a), to_m(f.b), f.normal) for f in self.obstacles.faces]
            if self.unknown == "occupied":
                safe = safe_free_mask(grid, r)
                segments += [(to_m(e.a), to_m(e.b), _free_normal(grid, safe, e.a, e.b))
                             for e in boundary_edges(grid, footprint_radius=r) if e.kind == "frontier"]
        faces = place_faces(self.v0, segments, self.placement, self.max_side_len)
        return Scene(self.v0, faces, rings, self.clearance, self.horizon)
