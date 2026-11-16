"""
Boundary of the known free space as straight edges, each labelled by what
lies behind it:

  obstacle -- an observed occupied cell (its length estimates the obstacle side);
  frontier -- unknown space (treated as occupied by the safety filter).

The free space is first shrunk by the robot footprint (cells closer than
``footprint_radius`` to any non-free cell are dropped), so the edges are for
a point robot.  Use ``footprint_radius=0`` to estimate obstacle side lengths
and the true footprint for the safety model.  Cells outside the map count as
unknown.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np
from contourpy import contour_generator
from scipy.ndimage import distance_transform_edt

from .occupancy import FREE, OCCUPIED, OccupancyGrid

__all__ = ["Edge", "safe_free_mask", "boundary_edges", "simplify_polyline", "snap_corners"]


@dataclass(frozen=True)
class Edge:
    a: np.ndarray
    b: np.ndarray
    kind: str                            # "obstacle" or "frontier"

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.b - self.a))


def _padded(grid: OccupancyGrid):
    """State padded by one unknown cell on every side, and its centre coordinates."""
    s = np.pad(grid.state(), 1, constant_values=0)
    xs, ys = grid.cell_centers()
    xs = np.concatenate([[xs[0] - grid.res], xs, [xs[-1] + grid.res]])
    ys = np.concatenate([[ys[0] - grid.res], ys, [ys[-1] + grid.res]])
    return s, xs, ys


def safe_free_mask(grid: OccupancyGrid, footprint_radius: float = 0.0) -> np.ndarray:
    """Free cells farther than ``footprint_radius`` from every non-free cell (unpadded shape)."""
    s, _, _ = _padded(grid)
    free = s == FREE
    if footprint_radius > 0:
        free &= distance_transform_edt(free) * grid.res > footprint_radius
    return free[1:-1, 1:-1]


def simplify_polyline(pts: np.ndarray, tol: float) -> np.ndarray:
    """Douglas-Peucker; keeps both end points."""
    keep = np.zeros(len(pts), bool)
    keep[[0, -1]] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        n = np.linalg.norm(ab)
        rel = pts[i + 1:j] - a
        d = np.abs(ab[0] * rel[:, 1] - ab[1] * rel[:, 0]) / n if n > 1e-12 else np.linalg.norm(rel, axis=1)
        k = int(np.argmax(d))
        if d[k] > tol:
            keep[i + 1 + k] = True
            stack += [(i, i + 1 + k), (i + 1 + k, j)]
    return pts[keep]


def snap_corners(pts: np.ndarray, closed: bool, short: float, min_turn: float = np.deg2rad(30)) -> np.ndarray:
    """Marching squares cuts every convex corner by a short chamfer edge.
    Replace each chamfer between two longer edges by the intersection of
    those edges' lines (when the turn exceeds ``min_turn``), so that side
    lengths are not underestimated by about one cell per end."""
    P = [p for p in (pts[:-1] if closed else pts)]
    changed = True
    while changed and len(P) >= 4:
        changed = False
        n = len(P)
        for k in range(n if closed else n - 1):
            idx = [(k + d) % n for d in (-1, 0, 1, 2)]
            if not closed and (k == 0 or k + 2 >= n):
                continue
            a, b, c, d = (P[i] for i in idx)
            u, v = b - a, d - c
            if (np.linalg.norm(c - b) >= short or np.linalg.norm(u) < short or np.linalg.norm(v) < short):
                continue
            cross = u[0] * v[1] - u[1] * v[0]
            if abs(cross) < np.sin(min_turn) * np.linalg.norm(u) * np.linalg.norm(v):
                continue
            s = ((c - a)[0] * v[1] - (c - a)[1] * v[0]) / cross
            x = a + s * u
            if np.linalg.norm(x - 0.5 * (b + c)) > 2 * short:
                continue
            P[idx[1]] = x
            del P[idx[2]]
            changed = True
            break
    P = np.array(P)
    if closed and len(P) >= 3:                  # the seam vertex may sit in the middle of a side
        a, b, c = P[-1], P[0], P[1]
        ac = c - a
        if abs(ac[0] * (b - a)[1] - ac[1] * (b - a)[0]) < 0.5 * short * np.linalg.norm(ac):
            P = P[1:]
    return np.vstack([P, P[:1]]) if closed else P


def boundary_edges(grid: OccupancyGrid, footprint_radius: float = 0.0, tol: "float | None" = None,
                   min_len: "float | None" = None) -> List[Edge]:
    """Straight edges of the boundary of ``safe_free_mask``.  ``tol``: polyline
    simplification tolerance (default one cell); ``min_len``: drop shorter
    edges (default half a cell)."""
    tol = grid.res if tol is None else tol
    min_len = 0.5 * grid.res if min_len is None else min_len
    s, xs, ys = _padded(grid)
    safe = np.pad(safe_free_mask(grid, footprint_radius), 1, constant_values=False)
    # kind of the nearest non-free cell, for every cell
    _, (ni, nj) = distance_transform_edt(s == FREE, return_indices=True)
    behind_occ = s[ni, nj] == OCCUPIED
    edges: List[Edge] = []
    for line in contour_generator(xs, ys, safe.astype(float)).lines(0.5):
        j = np.clip(np.rint((line[:, 0] - xs[0]) / grid.res).astype(int), 0, len(xs) - 1)
        i = np.clip(np.rint((line[:, 1] - ys[0]) / grid.res).astype(int), 0, len(ys) - 1)
        occ = behind_occ[i, j]
        # split where the label changes; consecutive runs share their end vertex
        cuts = np.flatnonzero(occ[1:] != occ[:-1]) + 1
        closed = len(cuts) == 0 and np.allclose(line[0], line[-1])
        for lo, hi in zip(np.r_[0, cuts], np.r_[cuts, len(line) - 1]):
            run = snap_corners(simplify_polyline(line[lo:hi + 1], tol), closed, short=2.5 * grid.res)
            kind = "obstacle" if occ[lo] else "frontier"
            edges += [Edge(a, b, kind) for a, b in zip(run[:-1], run[1:]) if np.linalg.norm(b - a) >= min_len]
    return edges
