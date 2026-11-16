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
      obstacle map of the last period (``self.obstacles``) also holds the
      corner list.
  "rings" -- every edge of the region's rings (the earlier behaviour).

FaST-BRT works in nondimensional units (v = 1, |w| <= 1); ``profile.units``
converts.  ``clearance`` (m) covers what the grid cannot resolve: the
marching-squares contour and the cell-centre distances can place an edge up
to about one cell beyond the truly free space.  ``h_max`` (m) caps how far a
primitive rectangle reaches off its edge: edges are cut to at most
2 h_max sqrt(alpha) (``fast_brt.primitive.max_side_len``).

Prototype: imports ``fast_brt`` from the environment (PYTHONPATH) until FaST-BRT
is a submodule.  ``RegionScene`` generalizes FaST-BRT's SceneModel (one
simple polygon) to a region with free-standing obstacles and to obstacle-only
edge sets; it belongs upstream in FaST-BRT.
"""

from __future__ import annotations

import numpy as np

from ..mapping import boundary_edges, extract_obstacles, free_region

__all__ = ["FastBrtGatekeeper", "make_region_scene"]


def _contains(ring: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Even-odd point-in-polygon for many points."""
    x, y = ring[:, 0], ring[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    px, py = xy[:, :1], xy[:, 1:2]
    dy = np.where(y2 == y, 1e-300, y2 - y)
    crosses = ((y > py) != (y2 > py)) & (px < x + (py - y) * (x2 - x) / dy)
    return np.count_nonzero(crosses, axis=1) % 2 == 1


def make_region_scene():
    """The RegionScene class (built lazily so that fast_brt is imported only when used)."""
    from fast_brt.filter.scene_model import SceneModel, split_edges
    from fast_brt.primitive import side_frame

    class RegionScene(SceneModel):
        """SceneModel over an explicit edge set.

        ``rings``: [outer, hole, ...] polygons of the free region, or None
        (no region: unknown space is free).  ``edges``: (a, b) segments that
        get a primitive each.  Modelled obstacle = (complement of the region,
        if any) U (edge rectangles); ``margin`` is the signed distance to it
        (positive = free), as in SceneModel."""

        def __init__(self, base, dtheta_V0_T, rings, edges, alpha: float, clearance: float,
                     max_side_len: "float | None" = None):
            self.base, self.dV = base, dtheta_V0_T
            self.rings = rings
            self.poly = rings[0] if rings else np.empty((0, 2))
            edges = [(np.asarray(a, float), np.asarray(b, float)) for a, b in edges
                     if np.linalg.norm(np.subtract(b, a)) > 1e-3]
            if max_side_len is not None:
                edges = split_edges(edges, max_side_len)
            self.frames = [side_frame(a, b, alpha) for a, b in edges]
            self.A = np.array([e[0] for e in edges]).reshape(-1, 2)
            self.B = np.array([e[1] for e in edges]).reshape(-1, 2)
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
                 horizon: float = 2.0,
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
        self.profile, self.units, self.alpha, self.unknown = profile, profile.units, alpha, unknown
        self.primitives = primitives
        self.obstacles = None                          # ObstacleMap of the last period ("faces")
        self._step_arc, self._Scene = step_arc, make_region_scene()
        res = profile.map.resolution
        self.clearance = (1.5 * res if clearance is None else clearance) / self.units.length
        self.max_side_len = max_side_len((res if h_max is None else h_max) / self.units.length, alpha)
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
            edges = [(a, b) for ring in rings for a, b in zip(ring, np.roll(ring, -1, axis=0))]
        else:
            self.obstacles = extract_obstacles(grid, offset=r)
            edges = [(to_m(f.a), to_m(f.b)) for f in self.obstacles.faces]
            if self.unknown == "occupied":
                edges += [(to_m(e.a), to_m(e.b)) for e in boundary_edges(grid, footprint_radius=r)
                          if e.kind == "frontier"]
        return self._Scene(self.base, self.dV, rings, edges, self.alpha, self.clearance, self.max_side_len)

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
