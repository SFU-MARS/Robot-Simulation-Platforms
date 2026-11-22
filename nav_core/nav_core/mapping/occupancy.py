"""
Known-pose occupancy grid, accumulated over the whole run.

Every scan ray marks the cells it crosses as passed and, if it hit something
within range, its end cell as hit.  A cell is updated at most once per scan,
and a hit outranks a pass in the same scan.  The environment is static, so
the free cells only grow (up to sensor noise).

Models (``model``):

  "counts" (default) -- each cell counts its hits and passes; it is occupied
      when hits / (hits + passes) >= ``tau``, free when observed below that,
      unknown when never observed.  A cell on an obstacle surface is partly
      free, so rays grazing its free part pass it far more often than rays
      end in it; a hit ratio well below one half keeps it occupied, while a
      stray hit in open space (1 hit, many passes) stays free.  Nothing is
      clipped, so evidence is never forgotten.
  "logodds" -- the classic additive log-odds with clipping: hits add
      ``l_occ``, passes add ``l_free``; occupied / free beyond +-``threshold``.
      Grazing passes outweigh hits on surface cells, which then flicker.

``graze`` (fraction of a cell, None: off): a cell with obstacle evidence (a
hit so far, or positive log-odds) is counted as passed only by a ray that
goes within ``graze * res`` of its centre, not by one clipping its corner.

On the occluded test world (README), the default (counts, tau 0.2, graze
0.3) cut edge-cell flicker from 38 / 80 / 112 to 2 / 15 / 27 events at 0 /
1 / 2 cm range noise, with fewer missed surface cells than log-odds.
"""

from __future__ import annotations

import numpy as np

__all__ = ["OccupancyGrid", "UNKNOWN", "FREE", "OCCUPIED"]

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class OccupancyGrid:
    """Cell (i, j) = (row along y, column along x); cell centres at
    origin + (j + 0.5, i + 0.5) * resolution."""

    def __init__(self, origin, size, resolution: float, model: str = "counts", tau: float = 0.2,
                 graze: "float | None" = 0.3, l_free: float = -0.7, l_occ: float = 0.85, l_min: float = -4.0,
                 l_max: float = 4.0, threshold: float = 0.6):
        if model not in ("counts", "logodds"):
            raise ValueError(f"model must be 'counts' or 'logodds', got {model!r}")
        self.origin = np.asarray(origin, float)
        self.res = float(resolution)
        self.nx, self.ny = (int(round(s / self.res)) for s in size)
        self.model, self.tau, self.graze = model, tau, graze
        self.hits = np.zeros((self.ny, self.nx))
        self.passes = np.zeros((self.ny, self.nx))
        self.logodds = np.zeros((self.ny, self.nx))
        self.l_free, self.l_occ, self.l_min, self.l_max, self.threshold = l_free, l_occ, l_min, l_max, threshold

    @classmethod
    def from_profile(cls, profile, **kw) -> "OccupancyGrid":
        return cls(profile.map.origin, profile.map.size, profile.map.resolution, **kw)

    def cell_centers(self):
        """(xs, ys): world coordinates of the column and row centres."""
        return (self.origin[0] + (np.arange(self.nx) + 0.5) * self.res,
                self.origin[1] + (np.arange(self.ny) + 0.5) * self.res)

    def world_to_cell(self, xy):
        """(i, j, inside) for world points ``xy`` (N, 2)."""
        ij = np.floor((np.atleast_2d(xy) - self.origin) / self.res).astype(int)
        j, i = ij[:, 0], ij[:, 1]
        return i, j, (i >= 0) & (i < self.ny) & (j >= 0) & (j < self.nx)

    def _evidence(self) -> np.ndarray:
        """Flat mask of cells with obstacle evidence (protected from grazing rays)."""
        return (self.hits > 0).reshape(-1) if self.model == "counts" else (self.logodds > 0).reshape(-1)

    def integrate_scan(self, pose, angles, ranges, range_max: float, range_min: float = 0.0) -> None:
        """``pose`` = (x, y, theta) of the sensor; ``angles`` relative to theta;
        ``ranges`` in metres, following sensor_msgs/LaserScan (REP 117):
        +inf or >= range_max is no return within range (the ray is free up to
        range_max); nan, -inf and readings below ``range_min`` are invalid and
        the ray is skipped, since it may hide a close obstacle."""
        x, y, th = pose
        ranges = np.asarray(ranges, float)
        angles = np.asarray(angles, float)
        valid = (ranges >= range_min) & ~np.isnan(ranges)                # nan / -inf / too close: skip
        ranges, angles = ranges[valid], angles[valid]
        hit = ranges < range_max
        r = np.where(hit, ranges, range_max)
        phi = th + angles
        step = 0.5 * self.res
        t = np.arange(0.0, range_max + step, step)                       # (S,)
        on_ray = t[None, :] < (r[:, None] - step)                        # stop half a cell short of the hit
        px = x + t[None, :] * np.cos(phi)[:, None]
        py = y + t[None, :] * np.sin(phi)[:, None]
        pts = np.stack([px[on_ray], py[on_ray]], 1)
        hits = np.stack([x + r[hit] * np.cos(phi[hit]), y + r[hit] * np.sin(phi[hit])], 1)
        occ = self._flat(hits)
        free = self._flat(pts)
        if self.graze is not None and len(pts):
            ray = np.broadcast_to(np.arange(len(phi))[:, None], on_ray.shape)[on_ray]
            i, j, inside = self.world_to_cell(pts)
            cells = (i * self.nx + j)[inside]
            cx = self.origin[0] + (j[inside] + 0.5) * self.res - x
            cy = self.origin[1] + (i[inside] + 0.5) * self.res - y
            c, sn = np.cos(phi[ray[inside]]), np.sin(phi[ray[inside]])
            central = np.unique(cells[np.abs(c * cy - sn * cx) < self.graze * self.res])
            free = free[~self._evidence()[free] | np.isin(free, central)]
        free = np.setdiff1d(free, occ)
        if self.model == "counts":
            self.passes.reshape(-1)[free] += 1
            self.hits.reshape(-1)[occ] += 1
        else:
            lo = self.logodds.reshape(-1)
            lo[free] = np.clip(lo[free] + self.l_free, self.l_min, self.l_max)
            lo[occ] = np.clip(lo[occ] + self.l_occ, self.l_min, self.l_max)

    def _flat(self, xy) -> np.ndarray:
        if len(xy) == 0:
            return np.empty(0, int)
        i, j, inside = self.world_to_cell(xy)
        return np.unique(i[inside] * self.nx + j[inside])

    def state(self) -> np.ndarray:
        """Per-cell UNKNOWN / FREE / OCCUPIED."""
        if self.model == "counts":
            n = self.hits + self.passes
            occupied = (n > 0) & (self.hits >= self.tau * n)
            s = np.full(n.shape, UNKNOWN, np.int8)
            s[(n > 0) & ~occupied] = FREE
            s[occupied] = OCCUPIED
            return s
        s = np.full(self.logodds.shape, UNKNOWN, np.int8)
        s[self.logodds <= -self.threshold] = FREE
        s[self.logodds >= self.threshold] = OCCUPIED
        return s
