"""
What the safety filters see each control period, in model units (v = 1, |w| <= 1; ``profile.units`` converts).
States in model units keep nav_core's order: (x, y, theta) for Dubins 3D, (x, y, theta, v) for Dubins 4D, whose V0
(fast-brt) is ordered (x, y, v, theta): ``Scene`` reorders for the queries.

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


def import_corner():
    """The module ``fast_brt.corner`` (the online corner correction, numpy only), found like ``import_query``."""
    import_query()
    import fast_brt.corner as corner
    return corner


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

    def __init__(self, v0, faces, rings, clearance: float, horizon: float, corners=(), rect_margin: bool = True):
        self.v0, self.faces = v0, faces
        self.rect_margin = rect_margin                # the primitive rectangles are part of the modelled obstacle
        self.four = v0.dims == 4                      # Dubins 4D: states (x, y, theta, v), V0 (x, y, v, theta)
        self.corners = list(corners)                  # fast_brt.corner.CornerWindow: corrected values at corners
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
        if len(self.faces) and (self.rect_margin or not self.rings):
            d = xy[None] - self._C[:, None]
            xl = self._cos[:, None] * d[..., 0] + self._sin[:, None] * d[..., 1]
            yl = -self._sin[:, None] * d[..., 0] + self._cos[:, None] * d[..., 1]
            rect = np.maximum(np.abs(xl) - self._HL[:, None], np.abs(yl) - self._HW[:, None]).min(axis=0)
            m = np.minimum(m, rect)
        return m

    def _v0_states(self, states: np.ndarray) -> np.ndarray:
        """States in V0's order: Dubins 4D (x, y, theta, v) -> (x, y, v, theta)."""
        states = np.atleast_2d(states)
        return states[:, [0, 1, 3, 2]] if self.four else states

    def value(self, states: np.ndarray) -> np.ndarray:
        """HJ value of the union of the primitives at states (N, 3), lowered at the corrected corners; inf without
        primitives."""
        q = self._v0_states(states)
        out = self.v0.value(q, self.faces)
        for w in self.corners:
            out = np.minimum(out, w.values(q))
        return out

    def hj_margin(self, states: np.ndarray) -> np.ndarray:
        """Distance margin of the union (model length units, ``PrimitiveV0.margin``): every state within it (same
        heading) is outside the BRT of every primitive; inf without primitives. Lowered at the corrected corners
        (their values are in the linear encoding)."""
        q = self._v0_states(states)
        out = self.v0.margin(q, self.faces)
        lipschitz = max(1.0, float(np.sqrt(self.v0.alpha)))
        for w in self.corners:
            out = np.minimum(out, w.values(q) / lipschitz)
        return out

    def hj_control(self, states: np.ndarray) -> np.ndarray:
        """Optimal turn rate w = sign(dV/dtheta) of what gives the value: the nearest primitive, or a corrected
        corner where it is lower; straight without primitives.  Dubins 4D: (N, 2) (a, w), a = a_max sign(dV/dv);
        without primitives no acceleration and straight."""
        states = np.atleast_2d(states)
        if self.four:
            if not len(self.faces):
                return np.zeros((len(states), 2))
            q = self._v0_states(states)
            res = self.v0.query(q, self.faces)
            control, best = res.control.copy(), res.value.copy()
            p = self.v0.dynamics_params
            a_max, w_max = float(p.get("aMax", 1.0)), float(p.get("wMax", 1.0))
            for w in self.corners:                    # a corrected corner gives the value: its (a, w), uMode max
                v = w.values(q)
                lower = v < best
                if lower.any():
                    control[lower, 0] = np.where(w.dv(q[lower]) < 0, -a_max, a_max)
                    control[lower, 1] = np.where(w.dtheta(q[lower]) < 0, -w_max, w_max)
                    best[lower] = v[lower]
            return control
        if not len(self.faces):
            return np.zeros(len(states))
        q = self.v0.query(states, self.faces)
        control, best = q.control[:, 0].copy(), q.value.copy()
        w_max = float(self.v0.dynamics_params.get("wMax", 1.0))
        for w in self.corners:
            v = w.values(states)
            lower = v < best
            if lower.any():
                control[lower] = np.where(w.dtheta(states[lower]) < 0, -w_max, w_max)
                best[lower] = v[lower]
        return control


class SceneFilter:
    """Common part of the filters: the profile, V0, units, and the scene of each period (``scene``).

    V0: ``v0`` if given, else the file in the environment variable ROBOT_SIM_V0, else the profile's
    ``base_value``; relative paths start at the folder that holds the profile's robots/ (``profile.root``).

    ``corners=True`` corrects the leaking corners near the robot (fast_brt.corner, docs/13; Dubins 4D: docs/15): the
    stationary local update on a window around each corner (spacing ``corner_dx``, model units; ``corner_theta``
    headings, default 30,
    None: as many as V0), solved when the corner appears or its
    faces (clipped to the window) move by more than ``corner_reuse`` (default 1.5 map cells); the scene's value,
    margin and HJ control use the corrected values.
    Only corners between obstacle faces by default; ``corner_frontier=True`` also corrects corners with a frontier
    edge (no difference in safety in the batches of docs/13, 23-32 % more solve time: the frontier moves every scan).
    ``corner_mode``: "sync" (solve at once), "delayed" (simulation: the result is usable once the measured solve
    time has passed in simulated time, solves queue on one simulated worker) or "thread" (a background thread).

    ``rect_margin``: whether the primitive rectangles are part of the modelled obstacle of ``Scene.margin`` (the
    witnesses' check) next to the complement of the region.  The region alone is already the configuration space
    of the known free space with round corners (a distance transform of the map), so it is safe on its own; the
    rectangles add conservatism, which is large for a large footprint: faces are extended by the footprint radius at
    their ends (square corners, and free ends of runs next to unknown space), and the rectangles' thickness reaches
    out of thin walls.  For the QBot (radius 0.285 m) a growing map then covers a robot that had stopped safely next
    to an obstacle, and it never has a witness again.  Default True for Dubins 3D (unchanged), False for Dubins 4D;
    filters without a region always use the rectangles."""

    uses_region = True                         # filters that only use the primitives set False: no region is built

    def __init__(self, profile, v0=None, unknown: str = "occupied", primitives: str = "faces",
                 placement: str = "inward", cut: bool = False, horizon: float = 2.0, sub_dt: "float | None" = None,
                 clearance: "float | None" = None, h_max: "float | None" = None, corners: bool = False,
                 corner_dx: "float | None" = None, corner_reuse: "float | None" = None, corner_frontier: bool = False,
                 corner_theta: "int | None" = 30, corner_warm: bool = True, corner_mode: str = "sync",
                 rect_margin: "bool | None" = None):
        profile.require_ready()
        if profile.dynamics not in ("dubins3d", "dubins4d"):
            raise NotImplementedError(f"{type(self).__name__} for dynamics {profile.dynamics!r}")
        self.four = profile.dynamics == "dubins4d"
        if sub_dt is None:
            sub_dt = 0.025 if self.four else 0.05       # model time: Burger 0.05 s, QBot 0.05 s (unit 2 s)
        # the modelled obstacle of the backup filter's witnesses: the complement of the region, and the primitive
        # rectangles too unless rect_margin is False (default: True for Dubins 3D, False for 4D, see the docstring)
        self.rect_margin = (not self.four) if rect_margin is None else rect_margin
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
        if self.four:
            u, ph = profile.units, profile.phys
            want = {"vMin": float(u.speed_to_model(ph["v_min"])), "vMax": 1.0,
                    "aMax": float(u.accel_to_model(ph["a_max"])), "wMax": 1.0}
            if (self.v0.dynamics != "dubins_4d" or p.get("uMode", "max") != "max"
                    or any(not np.isclose(float(p.get(k, np.nan)), v) for k, v in want.items())):
                raise ValueError(f"V0 must be the avoid value of the robot's Dubins 4D in model units ({want}, uMode "
                                 f"max), got {self.v0.dynamics} {p}")
            self.plant_model = profile.model.__class__(want["vMin"], 1.0, want["aMax"], 1.0)   # model units
        elif (self.v0.dynamics != "dubins_3d" or p.get("v", 1) != 1 or p.get("wMax", 1) != 1
                or p.get("uMode", "max") != "max"):
            raise ValueError(f"V0 must be the avoid value of the model Dubins 3D (v = wMax = 1, uMode max), got "
                             f"{self.v0.dynamics} {p}")
        else:
            self.plant_model = None
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
        # corner correction (docs/13): windows near the robot, a solved window reused while its faces, clipped to
        # the window, stay within corner_reuse (model units; None: 1.5 map cells, 3 cm on the Burger, 6 cm on the
        # QBot: the faces of a growing map jitter by a cell) of the ones it was solved for.
        # corner_dx None: fast_brt.corner.window_defaults (0.1 for 3D, 0.05 for 4D)
        if corner_reuse is None:
            corner_reuse = 1.5 * res / self.units.length
        if corners and self.four:
            from fast_brt.corner import numba_available
            if not numba_available():
                raise ImportError("the corner correction of the 4D car needs numba (1-6 s per window in numpy, about "
                                  "0.6-1.2 s compiled): pip install numba (0.56.4 with numpy 1.21 on ROS 2 Humble)")
        self.corners, self.corner_dx, self.corner_reuse = corners, corner_dx, corner_reuse
        self.corner_frontier, self.corner_theta, self.corner_warm = corner_frontier, corner_theta, corner_warm
        self.corner_warm_solves = 0
        self._n_obstacle_faces = None
        self._corner = import_corner() if corners else None
        if corner_mode not in ("sync", "delayed", "thread"):
            raise ValueError(f"corner_mode must be 'sync', 'delayed' or 'thread', got {corner_mode!r}")
        self.corner_mode = corner_mode
        self._slots = []                               # per corner: last window, its faces, the solve in progress
        self._period = 0
        self._worker_free = 0.0                        # "delayed": when the simulated worker is free again (s)
        self.corner_stale = 0                          # (corner, period) pairs without an up-to-date window
        if corners and corner_mode == "thread":
            from concurrent.futures import ThreadPoolExecutor
            self._corner_pool = ThreadPoolExecutor(max_workers=1)
        self.corner_solves = 0
        self.corner_seconds = 0.0

    def to_model(self, x) -> np.ndarray:
        """State in model units: (x, y, theta), or (x, y, theta, v) for Dubins 4D."""
        out = [*self.units.pos_to_model(x[:2]), x[2] % (2 * np.pi)]
        if self.four:
            out.append(float(self.units.speed_to_model(x[3])))
        return np.array(out)

    def control_to_model(self, u) -> np.ndarray:
        """A control in physical units -> model units, within the bounds: (w,) or (a, w)."""
        u = np.ravel(np.asarray(u, float))
        if self.four:
            hi = self.plant_model.u_high
            return np.clip([self.units.accel_to_model(u[0]), self.units.rate_to_model(u[1])], -hi, hi)
        return np.clip(self.units.rate_to_model(u[:1]), -1.0, 1.0)

    def controls_to_phys(self, us) -> np.ndarray:
        """Controls (k, m) in model units -> physical units: rates (k,) for Dubins 3D, (k, 2) for Dubins 4D."""
        us = np.asarray(us, float)
        if self.four:
            return np.stack([self.units.accel_to_phys(us[:, 0]), self.units.rate_to_phys(us[:, 1])], 1)
        return self.units.rate_to_phys(us.reshape(len(us), -1)[:, 0])

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
            self._n_obstacle_faces = len(segments)
            if self.unknown == "occupied":
                safe = safe_free_mask(grid, r)
                segments += [(to_m(e.a), to_m(e.b), _free_normal(grid, safe, e.a, e.b))
                             for e in boundary_edges(grid, footprint_radius=r) if e.kind == "frontier"]
        faces = place_faces(self.v0, segments, self.placement, self.max_side_len)
        return Scene(self.v0, faces, rings, self.clearance, self.horizon,
                     self.corner_windows(faces, self.units.pos_to_model(x[:2])), rect_margin=self.rect_margin)

    def corner_windows(self, faces, xy=None) -> list:
        """Corrected corner windows of the faces (``corners=True``) that can matter at ``xy`` (model units, the
        robot): within the horizon's reach plus the usable half width. Each corner has a slot with the last window
        solved for it; it is solved again (warm started from that window) only when its faces moved by more than
        ``corner_reuse``. ``corner_mode``: "sync" solves at once; "delayed" (simulation) solves at once but lets the
        robot use the result only after the measured solve time has passed in simulated time, one solve after the
        other as on one background worker; "thread" solves on a background thread (online). Until a new solve is
        ready, the slot's previous window is used (or none: the union only)."""
        if not self.corners:
            return []
        self._period += 1
        now = self._period * self.profile.ctrl_dt
        self._promote(now)
        if len(faces) < 2:
            self._slots = []
            return []
        corner = self._corner
        segments = corner.face_segments(faces)
        windows, slots = [], []
        for spec in corner.corner_windows(self.v0, faces, dx=self.corner_dx, n_theta=self.corner_theta):
            if not self.corner_frontier and self._n_obstacle_faces is not None:
                # only corners between obstacle faces, solved with the obstacle faces only: the frontier stays in the
                # union, but it moves with every scan and would make every window stale
                spec.pairs = [pr for pr in spec.pairs if max(pr[1], pr[2]) < self._n_obstacle_faces]
                spec.faces = tuple(k for k in spec.faces if k < self._n_obstacle_faces)
                if not spec.pairs:
                    continue
            center = np.asarray(spec.grid.center)
            if xy is not None and np.max(np.abs(center - xy)) > self.v0.reach + spec.grid.half:
                continue
            mine = _clip_segments(segments[list(spec.faces)], center, spec.grid.half)
            match = (spec.grid.half - corner.window_defaults(self.v0)["edge"]) / 2.0
            free = [sl for sl in self._slots if all(sl is not t for t in slots)]
            slot = min(free, key=lambda sl: float(np.max(np.abs(sl["center"] - center))), default=None)
            if slot is None or np.max(np.abs(slot["center"] - center)) > match:
                slot = {"center": center, "segs": None, "window": None, "job": None}
            slot["center"] = center
            fresh = slot["window"] is not None and _segments_close(slot["segs"], mine, self.corner_reuse)
            if not fresh and slot["job"] is None:
                self._submit(slot, faces, spec, mine, now)
                fresh = slot["job"] is None            # "sync": solved already
            self.corner_stale += not fresh
            if slot["window"] is not None:
                windows.append(slot["window"])
            slots.append(slot)
        self._slots = slots
        return windows

    def _submit(self, slot, faces, spec, mine, now) -> None:
        import time
        previous = slot["window"] if self.corner_warm else None
        self.corner_solves += 1
        self.corner_warm_solves += previous is not None
        if self.corner_mode == "thread":
            def solve():
                start = time.perf_counter()
                window = self._corner.solve_window(self.v0, faces, spec, previous=previous)
                return window, time.perf_counter() - start
            slot["job"] = {"future": self._corner_pool.submit(solve), "segs": mine}
            return
        start = time.perf_counter()
        window = self._corner.solve_window(self.v0, faces, spec, previous=previous)
        seconds = time.perf_counter() - start
        self.corner_seconds += seconds
        if self.corner_mode == "sync":
            slot["window"], slot["segs"] = window, mine
            return
        begin = max(now, self._worker_free)             # "delayed": one worker, in simulated time
        self._worker_free = begin + seconds
        slot["job"] = {"window": window, "segs": mine, "ready_at": self._worker_free}

    def _promote(self, now: float) -> None:
        """Move the finished solves into their slots."""
        for slot in self._slots:
            job = slot["job"]
            if job is None:
                continue
            if "future" in job:
                if not job["future"].done():
                    continue
                window, seconds = job["future"].result()
                self.corner_seconds += seconds
            elif job["ready_at"] > now:
                continue
            else:
                window = job["window"]
            slot["window"], slot["segs"], slot["job"] = window, job["segs"], None


def _clip_segments(segs: np.ndarray, center, half: float) -> np.ndarray:
    """The parts of the segments (m, 2, 2) inside the square |p - center| <= half (Liang-Barsky); segments outside
    are dropped."""
    out = []
    c = np.asarray(center, float)
    for a, b in segs:
        d = b - a
        t0, t1 = 0.0, 1.0
        for k in range(2):
            for p, q in ((-d[k], a[k] - (c[k] - half)), (d[k], (c[k] + half) - a[k])):
                if abs(p) < 1e-15:
                    if q < 0:
                        t0, t1 = 1.0, 0.0
                elif p < 0:
                    t0 = max(t0, q / p)
                else:
                    t1 = min(t1, q / p)
        if t0 <= t1:
            out.append((a + t0 * d, a + t1 * d))
    return np.array(out, float).reshape(-1, 2, 2)


def _point_segments(pts: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """Distance of every point (n, 2) to the nearest segment (m, 2, 2)."""
    a, ab = segs[:, 0], segs[:, 1] - segs[:, 0]
    ap = pts[:, None, :] - a[None]
    t = np.clip(np.sum(ap * ab[None], -1) / np.maximum(np.sum(ab * ab, -1), 1e-12)[None], 0.0, 1.0)
    return np.linalg.norm(ap - t[..., None] * ab[None], axis=-1).min(axis=1)


def _segments_close(a: np.ndarray, b: np.ndarray, tol: float) -> bool:
    """The two sets of segments are within tol of each other (Hausdorff distance, sampled every tol / 2): faces that
    split, merge or only jitter keep a solved window."""
    if len(a) == 0 or len(b) == 0:
        return len(a) == len(b)

    def samples(segs):
        pts = []
        for p, q in segs:
            n = int(np.ceil(np.linalg.norm(q - p) / (tol / 2.0))) + 1
            pts.append(p + np.linspace(0.0, 1.0, n)[:, None] * (q - p))
        return np.vstack(pts)
    return _point_segments(samples(a), b).max() <= tol and _point_segments(samples(b), a).max() <= tol
