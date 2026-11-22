"""
FaST-BRT witness gatekeeper as a ``nav_core.sim`` safety hook (Dubins 3D).

Each control period the accumulated map, shrunk by the robot footprint (so
the robot is a point), gives the edges of the known free space.  Every edge
the filter must respect gets one facing-side primitive; the gatekeeper
accepts the nominal turn rate only if a witness exists from the next state:
a rollout that keeps the clearance and ends on a free loiter circle.
Otherwise it executes the previously committed witness.

``unknown`` chooses what the filter must respect:

  "occupied" -- unknown space is an obstacle.  The region is the known free
      space around the robot; frontier edges are obstacle edges too.  Safe
      against obstacles hidden behind others, more conservative.
  "free" -- only detected obstacles count; unknown space is free.  The region
      is everything but the detected obstacles, each enclosed by its own ring.
      Less conservative, but safe only if every obstacle is seen before the
      robot enters its BRT (sensing range, no occlusion surprise).

In both modes the modelled obstacle for the witness check (``margin``) is the
complement of the region, so it covers every detected obstacle cell (grown
by the footprint) completely, not just its outline (README, Design
decision 1).  ``primitives`` chooses where the primitives for the HJ terms
come from:

  "faces" -- the obstacle layer: enclosing faces of every obstacle, shifted by
      the footprint, plus the frontier edges in the "occupied" mode.  The
      obstacle map of the last period (``self.obstacles``, stable ids) also
      holds the corner list; ``self.events`` lists what changed in that
      period (``ObstacleTracker``), e.g. for a leaking-corner update.
  "rings" -- every edge of the region's rings (the earlier behaviour).

FaST-BRT works in nondimensional units (v = 1, |w| <= 1); ``profile.units``
converts.  ``clearance`` (m) covers what the grid cannot resolve: the
marching-squares contour and the cell-centre distances can place an edge up
to about one cell beyond the truly free space.  ``h_max`` (m) caps how far a
primitive rectangle reaches off its edge: edges are cut to at most
2 h_max sqrt(alpha) (``fast_brt.primitive.max_side_len``).

``placement`` puts each primitive rectangle (length L, thickness L/sqrt(alpha))
on its edge:

  "inward" -- the rectangle's free-facing side lies on the edge and its
      thickness is inside the obstacle, so how far the primitive (and its BRT)
      reaches into free space depends on L only; long edges need no cutting.
      This is the intended use of the primitive.
  "centered" -- the edge is the rectangle's midline (FaST-BRT's ``side_frame``):
      half the thickness reaches into free space, which ``cut`` limits by cutting
      edges into short pieces (the earlier behaviour).

Prototype: imports ``fast_brt`` from the environment (PYTHONPATH) until FaST-BRT
is a submodule.  ``RegionScene`` generalizes FaST-BRT's SceneModel (one
simple polygon) to a region with free-standing obstacles and to obstacle-only
edge sets; it belongs upstream in FaST-BRT.
"""

from __future__ import annotations

import numpy as np

from ..mapping import ObstacleTracker, boundary_edges, extract_obstacles, free_region, safe_free_mask

__all__ = ["FastBrtGatekeeper", "make_region_scene", "primitive_frames"]


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


def primitive_frames(segments, alpha: float, placement: str = "inward", max_side_len: "float | None" = None):
    """Primitive frames (center, half length, half width, angle) for segments
    (a, b, n), n the unit normal of the segment pointing into free space.
    ``placement``: "inward" (the rectangle's free-facing side on the segment) or
    "centered" (the segment as the rectangle's midline).  ``max_side_len``
    (None: off) first cuts each segment into equal collinear pieces of at most
    that length."""
    if placement not in ("inward", "centered"):
        raise ValueError(f"placement must be 'inward' or 'centered', got {placement!r}")
    frames = []
    for a, b, n in segments:
        a, b, n = np.asarray(a, float), np.asarray(b, float), np.asarray(n, float)
        length = float(np.linalg.norm(b - a))
        if length <= 1e-9:
            continue
        k = 1 if max_side_len is None else max(1, int(np.ceil(length / max_side_len - 1e-9)))
        ts = np.linspace(0.0, 1.0, k + 1)
        beta = float(np.arctan2(*(b - a)[::-1]))
        for t0, t1 in zip(ts[:-1], ts[1:]):
            pa, pb = a + t0 * (b - a), a + t1 * (b - a)
            half_len = 0.5 * length * (t1 - t0)
            half_wid = half_len / np.sqrt(alpha)
            center = 0.5 * (pa + pb) - (half_wid * n if placement == "inward" else 0.0)
            frames.append((center, half_len, half_wid, beta))
    return frames


def make_region_scene():
    """The RegionScene class (built lazily so that fast_brt is imported only when used)."""
    from fast_brt.filter.scene_model import SceneModel

    class RegionScene(SceneModel):
        """SceneModel over an explicit set of primitive frames.

        ``rings``: [outer, hole, ...] polygons of the free region, or None
        (no region: unknown space is free).  ``frames``: primitive rectangles
        (center, half length, half width, angle), see ``primitive_frames``.
        Modelled obstacle = (complement of the region, if any) U (primitive
        rectangles); ``margin`` is the signed distance to it (positive = free),
        as in SceneModel."""

        def __init__(self, base, dtheta_V0_T, rings, frames, clearance: float):
            self.base, self.dV = base, dtheta_V0_T
            self.rings = rings
            self.poly = rings[0] if rings else np.empty((0, 2))
            self.frames = list(frames)
            self._C = np.array([f[0] for f in self.frames]).reshape(-1, 2)
            self._HL = np.array([f[1] for f in self.frames])
            self._HW = np.array([f[2] for f in self.frames])
            self._beta = np.array([f[3] for f in self.frames])
            self._cos, self._sin = np.cos(self._beta), np.sin(self._beta)
            if rings:
                ring_edges = [(a, b) for r in rings for a, b in zip(r, np.roll(r, -1, axis=0))]
                self._RA = np.array([e[0] for e in ring_edges])
                self._RB = np.array([e[1] for e in ring_edges])
            self.clearance = clearance
            self.T = base.horizon

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
            if len(self.frames):
                d = xy[None] - self._C[:, None]
                xl = self._cos[:, None] * d[..., 0] + self._sin[:, None] * d[..., 1]
                yl = -self._sin[:, None] * d[..., 0] + self._cos[:, None] * d[..., 1]
                rect = np.maximum(np.abs(xl) - self._HL[:, None], np.abs(yl) - self._HW[:, None]).min(axis=0)
                m = np.minimum(m, rect)
            return m

        # without any primitive nothing constrains the robot
        def side_values(self, states):
            if not self.frames:
                return np.full((1, len(np.atleast_2d(states))), np.inf)
            return super().side_values(states)

        def U_s(self, states, s):
            if not self.frames:
                return np.full(len(np.atleast_2d(states)), np.inf)
            return super().U_s(states, s)

        def hj_switch(self, states, w_max: float = 1.0):
            if not self.frames:
                return np.zeros(len(np.atleast_2d(states)))
            return super().hj_switch(states, w_max)

    return RegionScene


class FastBrtGatekeeper:
    def __init__(self, profile, unknown: str = "occupied", primitives: str = "faces", alpha: float = 20.0,
                 placement: str = "centered", cut: bool = True, horizon: float = 2.0,
                 sub_dt: float = 0.05, clearance: "float | None" = None, h_max: "float | None" = None,
                 search: str = "dive", budget: int = 300):
        from fast_brt.dynamics import step_arc
        from fast_brt.filter.gatekeeper import WitnessGatekeeper
        from fast_brt.hj import DubinsHJ, GridNP
        from fast_brt.primitive import BaseValue, max_side_len

        profile.require_ready()
        if profile.dynamics != "dubins3d":
            raise NotImplementedError(f"gatekeeper for dynamics {profile.dynamics!r}")
        if unknown not in ("occupied", "free"):
            raise ValueError(f"unknown must be 'occupied' or 'free', got {unknown!r}")
        if primitives not in ("faces", "rings"):
            raise ValueError(f"primitives must be 'faces' or 'rings', got {primitives!r}")
        if placement not in ("inward", "centered"):
            raise ValueError(f"placement must be 'inward' or 'centered', got {placement!r}")
        self.placement = placement
        self.profile, self.units, self.alpha, self.unknown = profile, profile.units, alpha, unknown
        self.primitives = primitives
        self.obstacles = None                          # ObstacleMap of the last period ("faces"), stable ids
        self.events = []                               # ObstacleEvents of the last period
        self.tracker = ObstacleTracker()
        self._step_arc, self._Scene = step_arc, make_region_scene()
        res = profile.map.resolution
        self.clearance = (1.5 * res if clearance is None else clearance) / self.units.length
        self.max_side_len = max_side_len((res if h_max is None else h_max) / self.units.length, alpha) if cut else None
        dyn = DubinsHJ(v_range=(1.0, 1.0), w_range=(-1.0, 1.0), role="avoid")
        self.base = BaseValue(GridNP.make((-6.0, 6.0), (-6.0, 6.0), 121, 121, 60), dyn, alpha, horizon, sub_dt)
        V = self.base.slices[-1].astype(float)
        self.dV = (np.roll(V, -1, axis=2) - np.roll(V, 1, axis=2)) / (2 * self.base.grid.dtheta)
        ctrl_dt = profile.ctrl_dt / self.units.time
        self.sub_dt = sub_dt
        self.n_sub = int(round(ctrl_dt / sub_dt))
        if not np.isclose(self.n_sub * sub_dt, ctrl_dt):
            raise ValueError(f"ctrl_dt {ctrl_dt} (model time) is not a multiple of sub_dt {sub_dt}")
        self.gate = WitnessGatekeeper(sub_dt, self.n_sub, int(round(horizon / sub_dt)), search=search, budget=budget)
        self.started = False
        self.steps = self.interventions = 0
        self.model = None                              # last scene, for plots

    def to_model(self, x) -> np.ndarray:
        return np.array([*self.units.pos_to_model(x[:2]), x[2] % (2 * np.pi)])

    def scene(self, x, grid):
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
        frames = primitive_frames(segments, self.alpha, self.placement, self.max_side_len)
        return self._Scene(self.base, self.dV, rings, frames, self.clearance)

    def __call__(self, x, w_nom: float, grid) -> np.ndarray:
        """ZOH turn rates (rad / s) for the ``n_sub`` sub-steps of this period."""
        model = self.scene(x, grid)
        self.model = model
        xm = self.to_model(x)
        if not self.started:
            self.gate.commit_initial(xm, model)        # RuntimeError: no witness at the start
            self.started = True
        wm = float(np.clip(self.units.rate_to_model(w_nom), -1.0, 1.0))
        x_next = xm.copy()[None]
        seg_m = np.inf
        for _ in range(self.n_sub):
            self._step_arc(x_next, np.array([[1.0, wm]]), self.sub_dt)
            seg_m = min(seg_m, model.margin(x_next)[0])
        ws, intervened = self.gate.filter(x_next, seg_m, wm, model)
        self.steps += 1
        self.interventions += int(intervened)
        return self.units.rate_to_phys(np.asarray(ws, float))
