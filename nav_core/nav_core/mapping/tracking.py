"""
Stable obstacle IDs across control periods, and detection events.

``extract_obstacles`` labels the obstacles of the map from scratch every
period, so its ids are not stable.  ``ObstacleTracker.update`` matches each
new obstacle to the previous period's by shared occupied cells (occupied
cells mostly persist: an obstacle grows as more of it is seen) and relabels
the obstacle map with stable ids.  It reports what changed:

  new      -- an obstacle that shares no cell with a tracked one;
  updated  -- same id, but its faces or corners changed (e.g. more of it seen);
  merged   -- it joins several tracked obstacles; it keeps the oldest id and
              lists the others in ``merged_from``;
  split    -- a part that lost its tracked obstacle to a larger part (noise);
  removed  -- a tracked obstacle with no occupied cell left (noise).

Consumers (the leaking-corner update, logs) can act on events only, instead
of on every obstacle every period.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

from .obstacles import Corner, Face, ObstacleMap

__all__ = ["ObstacleEvent", "ObstacleTracker"]


@dataclass(frozen=True)
class ObstacleEvent:
    t: float
    kind: str                            # new | updated | merged | split | removed
    obstacle: int                        # stable id
    faces: Tuple[Face, ...] = ()
    corners: Tuple[Corner, ...] = ()
    cells: int = 0
    merged_from: Tuple[int, ...] = ()

    def summary(self) -> str:
        if self.kind == "removed":
            return f"t={self.t:6.2f}s  obstacle {self.obstacle} removed"
        lengths = ", ".join(f"{f.length:.3f}" for f in sorted(self.faces, key=lambda f: -f.length))
        convex = sum(c.convex for c in self.corners)
        extra = f" (merged {list(self.merged_from)})" if self.merged_from else ""
        return (f"t={self.t:6.2f}s  obstacle {self.obstacle} {self.kind}{extra}: {self.cells} cells, "
                f"faces [{lengths}] m, {convex} convex / {len(self.corners) - convex} concave corners")

    def to_dict(self) -> dict:
        return {"t": self.t, "kind": self.kind, "obstacle": self.obstacle, "cells": self.cells,
                "merged_from": list(self.merged_from),
                "faces": [{"a": f.a.tolist(), "b": f.b.tolist(), "length": f.length, "normal": f.normal.tolist()}
                          for f in self.faces],
                "corners": [{"point": c.point.tolist(), "convex": c.convex, "turn": c.turn} for c in self.corners]}


@dataclass
class ObstacleTracker:
    """``tol`` (m): an obstacle is ``updated`` when its number of faces or of
    convex / concave corners changes, or a face endpoint moves by more than
    ``tol`` (default two cells).  ``min_cells``: obstacles smaller than this
    (stray hits, a first glimpse) are tracked but not reported until they
    grow or merge."""
    tol: "float | None" = None
    min_cells: int = 4
    next_id: int = 1
    labels: "np.ndarray | None" = None   # stable id per cell of the last update, 0 = not occupied
    _sig: Dict[int, tuple] = field(default_factory=dict)       # id -> state at its last report

    @staticmethod
    def _state(faces, corners) -> tuple:
        ends = np.array([np.r_[f.a, f.b] for f in faces]).reshape(-1, 4)
        return ends, (sum(c.convex for c in corners), sum(not c.convex for c in corners))

    @staticmethod
    def _changed(old, new, tol) -> bool:
        (e0, k0), (e1, k1) = old, new
        if k0 != k1 or len(e0) != len(e1):
            return True
        if not len(e1):
            return False
        # every new face must have an old face whose endpoints (either order) lie within tol
        d = np.minimum(np.maximum(np.linalg.norm(e1[:, None, :2] - e0[None, :, :2], axis=-1),
                                  np.linalg.norm(e1[:, None, 2:] - e0[None, :, 2:], axis=-1)),
                       np.maximum(np.linalg.norm(e1[:, None, :2] - e0[None, :, 2:], axis=-1),
                                  np.linalg.norm(e1[:, None, 2:] - e0[None, :, :2], axis=-1)))
        return bool(d.min(axis=1).max() > tol)

    def update(self, om: ObstacleMap, t: float = 0.0, res: float = 0.02) -> Tuple[ObstacleMap, List[ObstacleEvent]]:
        """(``om`` relabelled with stable ids, events of this period)."""
        tol = self.tol if self.tol is not None else 2 * res
        new_lab = om.labels
        prev = self.labels if self.labels is not None and self.labels.shape == new_lab.shape else np.zeros_like(new_lab)
        # overlap of every new component with every tracked id
        both = (new_lab > 0) & (prev > 0)
        pairs, counts = np.unique(np.stack([new_lab[both], prev[both]]), axis=1, return_counts=True)
        overlap: Dict[int, Dict[int, int]] = {}
        for (k, pid), c in zip(pairs.T, counts):
            overlap.setdefault(int(k), {})[int(pid)] = int(c)
        # a tracked id goes to the new component that holds most of its cells
        owner: Dict[int, int] = {}
        for k, ov in overlap.items():
            for pid, c in ov.items():
                if pid not in owner or c > overlap[owner[pid]][pid]:
                    owner[pid] = k
        mapping: Dict[int, int] = {}
        kinds: Dict[int, str] = {}
        merged: Dict[int, Tuple[int, ...]] = {}
        for k in range(1, om.n + 1):
            mine = sorted(pid for pid, kk in owner.items() if kk == k)
            if mine:
                mapping[k] = mine[0]                         # oldest id
                if len(mine) > 1:
                    kinds[k], merged[k] = "merged", tuple(mine[1:])
            else:
                mapping[k] = self.next_id
                self.next_id += 1
                kinds[k] = "split" if k in overlap else "new"
        stable = np.zeros_like(new_lab)
        for k, sid in mapping.items():
            stable[new_lab == k] = sid
        faces = [dataclasses.replace(f, obstacle=mapping[f.obstacle]) for f in om.faces]
        corners = [dataclasses.replace(c, obstacle=mapping[c.obstacle]) for c in om.corners]
        out = ObstacleMap(stable, om.n, faces, corners)

        events: List[ObstacleEvent] = []
        sig: Dict[int, tuple] = {}
        absorbed = {pid for m in merged.values() for pid in m}
        for k, sid in mapping.items():
            fs = tuple(f for f in faces if f.obstacle == sid)
            cs = tuple(c for c in corners if c.obstacle == sid)
            cells = int((stable == sid).sum())
            state = self._state(fs, cs)
            reported = sid in self._sig
            if cells < self.min_cells and kinds.get(k) != "merged":
                if reported:
                    sig[sid] = self._sig[sid]                  # keep the last reported state
                continue
            from_ids = tuple(p for p in merged.get(k, ()) if p in self._sig)   # reported ids it absorbed
            if from_ids:
                kind = "merged"
            elif not reported:                                 # first report, possibly after growing
                kind = "split" if kinds.get(k) == "split" else "new"
            else:                                              # absorbed nothing that was reported
                kind = "updated" if self._changed(self._sig[sid], state, tol) else None
            sig[sid] = state if kind else self._sig[sid]
            if kind:
                events.append(ObstacleEvent(t, kind, sid, fs, cs, cells, from_ids))
        for pid in self._sig:
            if pid not in sig and pid not in absorbed:
                events.append(ObstacleEvent(t, "removed", pid))
        self.labels, self._sig = stable, sig
        return out, events
