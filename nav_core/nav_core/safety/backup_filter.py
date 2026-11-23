"""
Backup filter (Dubins 3D) as a ``nav_core.sim`` safety hook.

Each control period the accumulated map, shrunk by the robot footprint (so
the robot is a point), gives the edges of the known free space.  Every edge
the filter must respect becomes one primitive; the filter accepts the nominal
turn rate only if a witness (backup plan, ``witness.py``) exists from the
next state: a rollout that keeps the clearance and ends on a free loiter
circle.  Otherwise it executes the previously committed witness.  Ported from
the witness gatekeeper of Hu-Hanyang/FaST-BRT.

``unknown`` chooses what the filter must respect:

  "occupied" -- unknown space is an obstacle.  The region is the known free
      space around the robot; frontier edges are obstacle edges too.  Safe
      against obstacles hidden behind others, more conservative.
  "free" -- only detected obstacles count; unknown space is free.  The region
      is everything but the detected obstacles, each enclosed by its own ring.
      Less conservative, but safe only if every obstacle is seen before the
      robot enters its BRT (sensing range, no occlusion surprise).

In both modes the modelled obstacle for the witness check (``Scene.margin``)
is the complement of the region together with the primitive rectangles, so it
covers every detected obstacle cell (grown by the footprint) completely, not
just its outline.  Safety rests on this geometry only; the HJ value of the
primitives proposes the first candidate of the search (``Scene.hj_control``).

``primitives`` chooses where the primitives come from:

  "faces" -- the obstacle layer: enclosing faces of every obstacle, shifted by
      the footprint, plus the frontier edges in the "occupied" mode.  The
      obstacle map of the last period (``self.obstacles``, stable ids) also
      holds the corner list; ``self.events`` lists what changed in that
      period (``ObstacleTracker``), e.g. for a leaking-corner update.
  "rings" -- every edge of the region's rings.

The HJ value comes from an offline V0 (``python -m fast_brt export-v0`` in
fast-brt), read by ``fast_brt.query.PrimitiveV0`` (numpy only): ``v0`` is a
``PrimitiveV0``, a V0 file, or None for the profile's ``base_value`` (a path
relative to the repository root).  V0 also sets alpha.  Model units: v = 1,
|w| <= 1 (``profile.units`` converts).  ``clearance`` (m) covers what the
grid cannot resolve: the marching-squares contour and the cell-centre
distances can place an edge up to about one cell beyond the truly free space.

``placement`` puts each primitive rectangle (length L, thickness L/sqrt(alpha))
on its edge:

  "inward" -- the rectangle's free-facing side lies on the edge and its
      thickness is inside the obstacle, so how far the primitive (and its BRT)
      reaches into free space depends on L only; long edges need no cutting.
  "centered" -- the edge is the rectangle's midline (the earlier behaviour):
      half the thickness reaches into free space, which ``cut`` limits by
      cutting edges into pieces of at most 2 h_max sqrt(alpha).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from ..mapping import ObstacleTracker, boundary_edges, extract_obstacles, free_region, safe_free_mask
from .witness import arc_step, find_witness, witness_slack

__all__ = ["BackupFilter", "Scene", "load_v0", "place_faces"]

REPO_ROOT = Path(__file__).resolve().parents[3]


def load_v0(v0):
    """A ``fast_brt.query.PrimitiveV0`` from a V0 file (relative paths from the repository root), or
    ``v0`` itself if it is one.  fast_brt is imported from the environment, else from the fast-brt
    submodule of this repository."""
    try:
        from fast_brt.query import PrimitiveV0
    except ImportError:
        submodule = REPO_ROOT / "fast-brt"
        if "fast_brt" in sys.modules or not (submodule / "fast_brt" / "query.py").is_file():
            raise ImportError("fast_brt.query is not importable: put SFU-MARS/fast-brt (branch robot-sim) on "
                              "PYTHONPATH, not Hu-Hanyang/FaST-BRT, or check out the fast-brt submodule") from None
        sys.path.insert(0, str(submodule))
        from fast_brt.query import PrimitiveV0
    if isinstance(v0, PrimitiveV0):
        return v0
    path = Path(v0).expanduser()
    path = path if path.is_absolute() else REPO_ROOT / path
    if not path.is_file():
        raise FileNotFoundError(f"no V0 file at {path}: export it with `python -m fast_brt export-v0 --config "
                                "configs/v0_dubins_3d.json` in fast-brt (odp environment) and copy it there")
    return PrimitiveV0.load(path)


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

    def hj_control(self, states: np.ndarray) -> np.ndarray:
        """Optimal turn rate of the nearest primitive, w = sign(dV/dtheta); straight without primitives."""
        states = np.atleast_2d(states)
        if not len(self.faces):
            return np.zeros(len(states))
        return self.v0.query(states, self.faces).control[:, 0]


class BackupFilter:
    def __init__(self, profile, v0=None, unknown: str = "occupied", primitives: str = "faces",
                 placement: str = "inward", cut: bool = False, horizon: float = 2.0, sub_dt: float = 0.05,
                 clearance: "float | None" = None, h_max: "float | None" = None, use_hj: bool = True):
        profile.require_ready()
        if profile.dynamics != "dubins3d":
            raise NotImplementedError(f"backup filter for dynamics {profile.dynamics!r}")
        if unknown not in ("occupied", "free"):
            raise ValueError(f"unknown must be 'occupied' or 'free', got {unknown!r}")
        if primitives not in ("faces", "rings"):
            raise ValueError(f"primitives must be 'faces' or 'rings', got {primitives!r}")
        if placement not in ("inward", "centered"):
            raise ValueError(f"placement must be 'inward' or 'centered', got {placement!r}")
        if v0 is None:
            if profile.base_value is None:
                raise ValueError(f"no V0: set base_value in robots/{profile.name}.yaml or pass v0")
            v0 = profile.base_value
        self.v0 = load_v0(v0)
        p = self.v0.dynamics_params
        if (self.v0.dynamics != "dubins_3d" or p.get("v", 1) != 1 or p.get("wMax", 1) != 1
                or p.get("uMode", "max") != "max"):
            raise ValueError(f"V0 must be the avoid value of the model Dubins 3D (v = wMax = 1, uMode max), got "
                             f"{self.v0.dynamics} {p}")
        self.alpha = self.v0.alpha
        self.profile, self.units, self.unknown, self.primitives = profile, profile.units, unknown, primitives
        self.placement, self.use_hj = placement, use_hj
        self.obstacles = None                          # ObstacleMap of the last period ("faces"), stable ids
        self.events = []                               # ObstacleEvents of the last period
        self.tracker = ObstacleTracker()
        res = profile.map.resolution
        self.clearance = (1.5 * res if clearance is None else clearance) / self.units.length
        if self.clearance < witness_slack(sub_dt):
            raise ValueError(f"clearance {self.clearance:g} (model units) < witness sampling slack "
                             f"{witness_slack(sub_dt):.4g}")
        h = (res if h_max is None else h_max) / self.units.length
        self.max_side_len = 2.0 * h * np.sqrt(self.alpha) if cut else None
        ctrl_dt = profile.ctrl_dt / self.units.time
        self.horizon, self.sub_dt = horizon, sub_dt
        self.n_sub = int(round(ctrl_dt / sub_dt))
        if not np.isclose(self.n_sub * sub_dt, ctrl_dt):
            raise ValueError(f"ctrl_dt {ctrl_dt} (model time) is not a multiple of sub_dt {sub_dt}")
        self.K = int(round(horizon / sub_dt))
        self.plan = None                               # committed witness, padded to K steps with its loiter turn
        self.loiter = 0.0
        self.started = False
        self.steps = self.interventions = 0
        self.model = None                              # last scene, for plots

    def to_model(self, x) -> np.ndarray:
        return np.array([*self.units.pos_to_model(x[:2]), x[2] % (2 * np.pi)])

    def scene(self, x, grid) -> Scene:
        """The scene (model units) the filter sees at pose ``x``."""
        to_m = self.units.pos_to_model
        r = self.profile.footprint_radius
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

    def _commit(self, found) -> None:
        plan, self.loiter = found
        if plan.size < self.K:
            plan = np.concatenate([plan, np.full(self.K - plan.size, self.loiter)])
        self.plan = plan

    def __call__(self, x, w_nom: float, grid) -> np.ndarray:
        """ZOH turn rates (rad / s) for the ``n_sub`` sub-steps of this period."""
        model = self.scene(x, grid)
        self.model = model
        xm = self.to_model(x)
        if not self.started:
            found = find_witness(xm, model, self.sub_dt, self.use_hj)
            if found is None:
                raise RuntimeError("no witness at the start")
            self._commit(found)
            self.started = True
        wm = float(np.clip(self.units.rate_to_model(w_nom), -1.0, 1.0))
        x_next = xm[None].copy()
        seg_margin = np.inf
        for _ in range(self.n_sub):
            arc_step(x_next, np.array([wm]), self.sub_dt)
            seg_margin = min(seg_margin, model.margin(x_next[:, :2])[0])
        found = find_witness(x_next[0], model, self.sub_dt, self.use_hj) if seg_margin > model.clearance else None
        self.steps += 1
        if found is not None:                          # accept the nominal step, its witness is the new plan
            self._commit(found)
            return self.units.rate_to_phys(np.full(self.n_sub, wm))
        self.interventions += 1                        # execute the committed plan, keep loitering after it
        ws = self.plan[:self.n_sub]
        self.plan = np.concatenate([self.plan[self.n_sub:], np.full(self.n_sub, self.loiter)])
        return self.units.rate_to_phys(np.asarray(ws, float))
