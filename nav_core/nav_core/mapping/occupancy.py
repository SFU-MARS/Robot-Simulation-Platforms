"""
Known-pose log-odds occupancy grid, accumulated over the whole run.

Cells start unknown (log-odds 0).  Every scan ray marks the cells it crosses
free and, if it hit something within range, its end cell occupied.  A cell
is updated at most once per scan, and a hit outranks a pass in the same scan.
The environment is static, so the free cells only grow (up to sensor noise).
"""

from __future__ import annotations

import numpy as np

__all__ = ["OccupancyGrid", "UNKNOWN", "FREE", "OCCUPIED"]

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class OccupancyGrid:
    """Cell (i, j) = (row along y, column along x); cell centres at
    origin + (j + 0.5, i + 0.5) * resolution."""

    def __init__(self, origin, size, resolution: float, l_free: float = -0.4, l_occ: float = 0.85,
                 l_min: float = -4.0, l_max: float = 4.0, threshold: float = 0.6):
        self.origin = np.asarray(origin, float)
        self.res = float(resolution)
        self.nx, self.ny = (int(round(s / self.res)) for s in size)
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

    def integrate_scan(self, pose, angles, ranges, range_max: float) -> None:
        """``pose`` = (x, y, theta) of the sensor; ``angles`` relative to theta;
        ``ranges`` in metres (inf / nan / >= range_max: no hit within range)."""
        x, y, th = pose
        ranges = np.asarray(ranges, float)
        hit = np.isfinite(ranges) & (ranges < range_max)
        r = np.where(hit, ranges, range_max)
        phi = th + np.asarray(angles, float)
        step = 0.5 * self.res
        t = np.arange(0.0, range_max + step, step)                       # (S,)
        on_ray = t[None, :] < (r[:, None] - step)                        # stop half a cell short of the hit
        px = x + t[None, :] * np.cos(phi)[:, None]
        py = y + t[None, :] * np.sin(phi)[:, None]
        pts = np.stack([px[on_ray], py[on_ray]], 1)
        hits = np.stack([x + r[hit] * np.cos(phi[hit]), y + r[hit] * np.sin(phi[hit])], 1)
        occ = self._flat(hits)
        free = np.setdiff1d(self._flat(pts), occ)
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
        s = np.full(self.logodds.shape, UNKNOWN, np.int8)
        s[self.logodds <= -self.threshold] = FREE
        s[self.logodds >= self.threshold] = OCCUPIED
        return s
