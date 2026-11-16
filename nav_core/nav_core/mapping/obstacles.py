"""
Obstacle layer (README, Design decisions 1-4): obstacles, their faces and
corners, from the accumulated occupancy grid.

  obstacle -- a connected component of occupied cells (8-connected, after
              bridging gaps of ``bridge`` cells); touching objects are one.
  face     -- a straight enclosing line along the free-facing boundary of
              one obstacle; its length estimates that side.
  corner   -- where two consecutive faces of an obstacle meet at an angle;
              ``convex`` w.r.t. the obstacle (where leaking corners occur).

Enclosing.  A face is shifted towards the free space by the largest
deviation of the boundary points it replaces, so every point of the
obstacle's free-facing boundary lies on the obstacle side of its face.  With
``offset`` > 0 (e.g. the footprint radius) faces are shifted further and
their free ends extended by ``offset``: the configuration-space obstacle,
with square instead of rounded corners.

Faces are taken from the map, not from single scans, so there is no data
association across scans: an obstacle grows in the map as more of it is seen.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
from contourpy import contour_generator
from scipy.ndimage import binary_dilation, distance_transform_edt, label

from .boundary import _padded, simplify_polyline, snap_corners
from .occupancy import FREE, OCCUPIED, OccupancyGrid

__all__ = ["Face", "Corner", "ObstacleMap", "label_obstacles", "extract_obstacles"]


@dataclass(frozen=True)
class Face:
    obstacle: int                        # obstacle id (>= 1)
    a: np.ndarray
    b: np.ndarray
    normal: np.ndarray                   # unit normal pointing into free space

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.b - self.a))


@dataclass(frozen=True)
class Corner:
    obstacle: int
    faces: Tuple[int, int]               # indices into ObstacleMap.faces, in boundary order
    point: np.ndarray
    convex: bool                         # w.r.t. the obstacle
    turn: float                          # rad, angle between the two face directions


@dataclass
class ObstacleMap:
    labels: np.ndarray                   # (ny, nx) obstacle id per cell, 0 = not occupied
    n: int                               # number of obstacles
    faces: List[Face] = field(default_factory=list)
    corners: List[Corner] = field(default_factory=list)

    def faces_of(self, obstacle: int) -> List[Face]:
        return [f for f in self.faces if f.obstacle == obstacle]


def label_obstacles(grid: OccupancyGrid, bridge: int = 1):
    """(labels, n): connected components of occupied cells; gaps up to
    ``bridge`` cells (e.g. unobserved cells along a wall) do not split one."""
    occ = grid.state() == OCCUPIED
    grown = binary_dilation(occ, iterations=bridge) if bridge > 0 else occ
    lab, _ = label(grown, structure=np.ones((3, 3), int))
    lab[~occ] = 0
    used = np.unique(lab[lab > 0])                       # renumber 1..n
    remap = np.zeros(lab.max() + 1, int)
    remap[used] = np.arange(1, len(used) + 1)
    return remap[lab], len(used)


def _fill_short_gaps(ids: np.ndarray, max_gap: int, closed: bool) -> np.ndarray:
    """Relabel runs of at most ``max_gap`` vertices that sit between two runs
    of the same obstacle (e.g. a few unobserved cells along a wall)."""
    ids = ids.copy()
    n = len(ids)
    if n == 0:
        return ids
    cuts = np.flatnonzero(ids[1:] != ids[:-1]) + 1
    runs = list(zip(np.r_[0, cuts], np.r_[cuts, n]))
    if len(runs) <= 2:
        return ids
    for k, (lo, hi) in enumerate(runs):
        if hi - lo > max_gap or (not closed and k in (0, len(runs) - 1)):
            continue
        prev = ids[runs[k - 1][0]]
        nxt = ids[runs[(k + 1) % len(runs)][0]]
        if prev == nxt and prev > 0:
            ids[lo:hi] = prev
    return ids


def _intersect(p, d, q, e):
    """Intersection of lines p + s d and q + t e (None if parallel)."""
    cross = d[0] * e[1] - d[1] * e[0]
    if abs(cross) < 1e-12:
        return None
    s = ((q - p)[0] * e[1] - (q - p)[1] * e[0]) / cross
    return p + s * d


def _merge_shallow(V: np.ndarray, pts: np.ndarray, closed: bool, min_turn: float, max_dev: float) -> np.ndarray:
    """Drop polyline vertices where the line turns by less than ``min_turn``,
    as long as every boundary point stays within ``max_dev`` of the merged
    chord (range noise makes a straight wall jagged; a real curve keeps its
    vertices).  Shallowest turn first."""
    V = [v for v in V]
    while len(V) > (3 if closed else 2):
        n = len(V)
        best, best_turn = None, min_turn
        for k in (range(n) if closed else range(1, n - 1)):
            a, v, b = V[k - 1], V[k], V[(k + 1) % n]
            d, e = v - a, b - v
            turn = float(np.arccos(np.clip(d @ e / max(np.linalg.norm(d) * np.linalg.norm(e), 1e-12), -1, 1)))
            if turn >= best_turn:
                continue
            ab = b - a
            L = np.linalg.norm(ab)
            rel = pts - a
            t = rel @ ab / max(L * L, 1e-12)
            near = (t > 0) & (t < 1) & (np.abs(ab[0] * rel[:, 1] - ab[1] * rel[:, 0]) / max(L, 1e-12) < 4 * max_dev)
            dev = np.abs(ab[0] * rel[near, 1] - ab[1] * rel[near, 0]) / max(L, 1e-12)
            if dev.size == 0 or dev.max() <= max_dev:
                best, best_turn = k, turn
        if best is None:
            break
        del V[best]
    return np.array(V)


def _run_faces(pts: np.ndarray, closed: bool, free_at, res: float, tol: float, offset: float,
               min_turn: float):
    """Enclosing faces of one boundary run of one obstacle: list of
    (a, b, normal), and (convex, turn) between consecutive faces."""
    poly = snap_corners(simplify_polyline(np.vstack([pts, pts[:1]]) if closed else pts, tol), closed,
                        short=2.5 * res)
    V = poly[:-1] if closed else poly
    V = V[np.r_[True, np.linalg.norm(np.diff(V, axis=0), axis=1) > 1e-9]]
    if closed and len(V) > 1 and np.linalg.norm(V[-1] - V[0]) <= 1e-9:
        V = V[:-1]
    V = _merge_shallow(V, pts, closed, min_turn, 2.0 * tol)
    m = len(V) if closed else len(V) - 1
    if m < 1:
        return [], []
    segs = [(V[k], V[(k + 1) % len(V)]) for k in range(m)]
    # free-side normal of every segment: probe both sides of its midpoint
    normals = []
    for a, b in segs:
        d = (b - a) / max(np.linalg.norm(b - a), 1e-12)
        n = np.array([-d[1], d[0]])
        mid = 0.5 * (a + b)
        score = free_at(mid + 0.75 * res * n) - free_at(mid - 0.75 * res * n)
        normals.append(n if score >= 0 else -n)
    # assign each boundary point to its nearest segment; shift so that all lie behind the line
    A = np.array([s[0] for s in segs])
    B = np.array([s[1] for s in segs])
    ab = B - A
    ap = pts[:, None, :] - A[None]
    t = np.clip(np.sum(ap * ab[None], -1) / np.maximum(np.sum(ab * ab, -1), 1e-12)[None], 0, 1)
    owner = np.linalg.norm(ap - t[..., None] * ab[None], axis=-1).argmin(1)
    lines = []
    for k, (a, b) in enumerate(segs):
        n = normals[k]
        dev = (pts[owner == k] - a) @ n if np.any(owner == k) else np.zeros(1)
        shift = max(0.0, float(dev.max())) + offset
        lines.append((a + shift * n, b - a, n))
    # vertices = intersections of consecutive shifted lines
    nv = len(lines) if closed else len(lines) + 1
    verts = [None] * nv
    flags = []
    for k in range(len(lines) if closed else len(lines) - 1):
        (p, d, n1), (q, e, _) = lines[k], lines[(k + 1) % len(lines)]
        turn = float(np.arccos(np.clip(d @ e / max(np.linalg.norm(d) * np.linalg.norm(e), 1e-12), -1, 1)))
        x = _intersect(p, d, q, e)
        if x is None or turn < min_turn:
            x = 0.5 * (p + d + q)                        # nearly collinear: meet halfway
        verts[(k + 1) % nv] = x
        flags.append((bool((e @ n1) < 0), turn))
    if not closed:                                       # free ends: extend by the offset
        p, d, _ = lines[0]
        verts[0] = p - offset * d / np.linalg.norm(d)
        q, e, _ = lines[-1]
        verts[-1] = q + e + offset * e / np.linalg.norm(e)
    faces = [(verts[k], verts[(k + 1) % nv], lines[k][2]) for k in range(len(lines))]
    return faces, flags


def extract_obstacles(grid: OccupancyGrid, offset: float = 0.0, tol: "float | None" = None,
                      min_len: "float | None" = None, min_turn_deg: float = 20.0, bridge: int = 1,
                      max_gap: int = 4) -> ObstacleMap:
    """Obstacles, enclosing faces and corners of the accumulated map.
    ``tol``: line-fit tolerance (default 1.5 cells: with ~1 cm range noise and
    a few scans per place; ~2 cells for 2 cm noise); ``min_len``: drop
    shorter faces (default one cell); ``max_gap``: unobserved stretches of
    at most this many boundary vertices between two parts of the same
    obstacle are bridged (one face across them)."""
    res = grid.res
    tol = 1.5 * res if tol is None else tol
    min_len = res if min_len is None else min_len
    min_turn = np.deg2rad(min_turn_deg)
    lab, n = label_obstacles(grid, bridge)
    s, xs, ys = _padded(grid)
    free = s == FREE
    labp = np.pad(lab, 1)
    _, (ni, nj) = distance_transform_edt(free, return_indices=True)
    behind = labp[ni, nj]                                # obstacle id behind each cell, 0 = unknown

    def free_at(xy) -> float:
        j = int(np.floor((xy[0] - xs[0]) / res + 0.5))
        i = int(np.floor((xy[1] - ys[0]) / res + 0.5))
        return float(0 <= i < free.shape[0] and 0 <= j < free.shape[1] and free[i, j])

    out = ObstacleMap(lab, n)
    for line in contour_generator(xs, ys, free.astype(float)).lines(0.5):
        closed = len(line) > 3 and np.allclose(line[0], line[-1])
        pts = line[:-1] if closed else line
        j = np.clip(np.rint((pts[:, 0] - xs[0]) / res).astype(int), 0, len(xs) - 1)
        i = np.clip(np.rint((pts[:, 1] - ys[0]) / res).astype(int), 0, len(ys) - 1)
        ids = _fill_short_gaps(behind[i, j], max_gap, closed)
        cuts = np.flatnonzero(ids[1:] != ids[:-1]) + 1
        if closed and len(cuts) and ids[0] == ids[-1]:   # rotate so that no run wraps around the seam
            pts, ids = np.roll(pts, -cuts[0], axis=0), np.roll(ids, -cuts[0])
            cuts = np.flatnonzero(ids[1:] != ids[:-1]) + 1
        whole = closed and len(cuts) == 0
        for lo, hi in zip(np.r_[0, cuts], np.r_[cuts, len(pts)]):
            oid = int(ids[lo])
            # an open run takes the next run's first vertex, so that faces reach the frontier
            if whole:
                run = pts
            elif hi < len(pts):
                run = pts[lo:hi + 1]
            else:
                run = np.vstack([pts[lo:], pts[:1]]) if closed else pts[lo:]
            if oid == 0 or len(run) < 2:
                continue
            faces, flags = _run_faces(run, whole, free_at, res, tol, offset, min_turn)
            index = {}
            for k, (a, b, nrm) in enumerate(faces):
                if np.linalg.norm(b - a) >= min_len:
                    index[k] = len(out.faces)
                    out.faces.append(Face(oid, a, b, nrm))
            for k, (cvx, turn) in enumerate(flags):
                k2 = (k + 1) % len(faces)
                if turn >= min_turn and k in index and k2 in index:
                    out.corners.append(Corner(oid, (index[k], index[k2]), faces[k][1], cvx, turn))
    return out
