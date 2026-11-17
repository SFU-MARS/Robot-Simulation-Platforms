"""
Map-aware MPPI nominal controller (Dubins 3D, constant speed, control = turn
rate).  No safety guarantee: it is optimistic about unknown space and only
steers the safety filter's input towards the goal and away from obstacles.

Costs, from the accumulated map (``CostMap``, rebuilt when the map changes):

  cost-to-go -- geodesic distance to the goal through cells that are not
                within the footprint of an occupied cell (unknown counts as
                traversable), so the controller goes around obstacles
                instead of pushing straight at the goal;
  obstacles  -- a large penalty inside the footprint of an occupied cell and
                a soft one close to it.

The dynamics enter only through ``step`` (vectorized), so another robot can
supply its own model (e.g. from FaST-BRT).
"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from ..mapping import OCCUPIED, OccupancyGrid

__all__ = ["CostMap", "MPPI", "unicycle_step_batch"]


def unicycle_step_batch(z: np.ndarray, v: float, w: np.ndarray, dt: float) -> np.ndarray:
    """Exact constant-speed unicycle step for every row of ``z`` (N, 3)."""
    th = z[:, 2]
    th1 = th + w * dt
    straight = np.abs(w) < 1e-9
    r = np.where(straight, 0.0, v / np.where(straight, 1.0, w))
    dx = np.where(straight, v * dt * np.cos(th), r * (np.sin(th1) - np.sin(th)))
    dy = np.where(straight, v * dt * np.sin(th), r * (np.cos(th) - np.cos(th1)))
    return np.stack([z[:, 0] + dx, z[:, 1] + dy, th1], 1)


class CostMap:
    """Obstacle distance and geodesic cost-to-go on the map grid (metres)."""

    def __init__(self, grid: OccupancyGrid, goal, footprint_radius: float):
        self.origin, self.res = grid.origin.copy(), grid.res
        st = grid.state()
        occ = st == OCCUPIED
        self.obst = distance_transform_edt(~occ) * grid.res if occ.any() else np.full(st.shape, np.inf)
        ok = self.obst > footprint_radius
        ny, nx = st.shape
        idx = np.arange(ny * nx).reshape(ny, nx)
        rows, cols, wts = [], [], []
        for di, dj, w in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2))):
            src = (slice(0, ny - di), slice(max(0, -dj), nx - max(0, dj)))            # cell (i, j)
            dst = (slice(di, ny), slice(max(0, -dj) + dj, nx - max(0, dj) + dj))     # cell (i + di, j + dj)
            m = ok[src] & ok[dst]
            rows.append(idx[src][m]); cols.append(idx[dst][m]); wts.append(np.full(m.sum(), w * grid.res))
        graph = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                           shape=(ny * nx, ny * nx)).tocsr()
        gi, gj = self._cell(np.atleast_2d(goal))
        gi, gj = int(np.clip(gi[0], 0, ny - 1)), int(np.clip(gj[0], 0, nx - 1))
        if not ok[gi, gj]:                                   # goal too close to an obstacle: nearest traversable cell
            cand = np.argwhere(ok)
            if len(cand):
                gi, gj = cand[np.argmin(np.hypot(cand[:, 0] - gi, cand[:, 1] - gj))]
        d = dijkstra(graph, directed=False, indices=int(idx[gi, gj]))
        self.c2g = d.reshape(ny, nx)
        self.c2g[~np.isfinite(self.c2g)] = 1e3               # unreachable
        self.shape = st.shape

    def _cell(self, xy):
        ij = np.floor((xy - self.origin) / self.res).astype(int)
        return ij[:, 1], ij[:, 0]

    def lookup(self, xy):
        """(cost-to-go, obstacle distance) at points ``xy`` (N, 2); outside the map: blocked."""
        i, j = self._cell(xy)
        inside = (i >= 0) & (i < self.shape[0]) & (j >= 0) & (j < self.shape[1])
        i, j = np.clip(i, 0, self.shape[0] - 1), np.clip(j, 0, self.shape[1] - 1)
        return np.where(inside, self.c2g[i, j], 1e3), np.where(inside, self.obst[i, j], 0.0)


class MPPI:
    """``control(x, goal, grid) -> w`` (rad / s)."""

    def __init__(self, v: float, w_max: float, footprint_radius: float, horizon: int = 30, dt: float = 0.1,
                 n_samples: int = 256, sigma: float = 0.6, lam: float = 0.05, margin: float = 0.03,
                 step: Optional[Callable] = None, seed: int = 0):
        self.v, self.w_max, self.r, self.margin = v, w_max, footprint_radius, margin
        self.H, self.dt, self.N, self.sigma, self.lam = horizon, dt, n_samples, sigma * w_max, lam
        self.step = step or (lambda z, w, dt: unicycle_step_batch(z, self.v, w, dt))
        self.u = np.zeros(horizon)
        self.rng = np.random.default_rng(seed)
        self._key, self.costmap = None, None

    def _costmap(self, grid, goal) -> CostMap:
        key = (tuple(np.round(goal, 6)), (grid.state() == OCCUPIED).tobytes())
        if key != self._key:
            self._key, self.costmap = key, CostMap(grid, goal, self.r)
        return self.costmap

    def control(self, x, goal, grid=None) -> float:
        if grid is None:
            raise ValueError("MPPI needs the map")
        cm = self._costmap(grid, goal)
        U = np.clip(self.u[None] + self.rng.normal(0.0, self.sigma, (self.N, self.H)), -self.w_max, self.w_max)
        U[0] = self.u                                     # keep the previous plan as a sample
        z = np.repeat(np.asarray(x, float)[None], self.N, axis=0)
        cost = np.zeros(self.N)
        for h in range(self.H):
            z = self.step(z, U[:, h], self.dt)
            c2g, d = cm.lookup(z[:, :2])
            cost += 0.1 * c2g + 1e3 * (d <= self.r) + 0.5 * np.exp(-(d - self.r) / self.margin)
        cost += 2.0 * c2g                                  # terminal cost-to-go
        w = np.exp(-(cost - cost.min()) / self.lam)
        w /= w.sum()
        self.u = np.clip(w @ U, -self.w_max, self.w_max)
        out = float(self.u[0])
        self.u = np.r_[self.u[1:], self.u[-1]]
        return out
