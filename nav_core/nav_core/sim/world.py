"""
Ground-truth 2D world for the ROS-free simulator: oriented boxes in the scan
plane, read from the same Gazebo SDF world the ROS 2 runs use.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List

import numpy as np

__all__ = ["Box", "World"]


@dataclass(frozen=True)
class Box:
    center: tuple            # (x, y), m
    half: tuple              # (half length along local x, half width along local y), m
    yaw: float = 0.0         # rad

    def to_local(self, xy: np.ndarray) -> np.ndarray:
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        d = np.atleast_2d(xy) - np.asarray(self.center)
        return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)

    def distance(self, xy: np.ndarray) -> np.ndarray:
        """Signed distance to the box (negative inside)."""
        q = np.abs(self.to_local(xy)) - np.asarray(self.half)
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        return outside + np.minimum(q.max(1), 0.0)


def _pose(el) -> np.ndarray:
    """(x, y, z, roll, pitch, yaw) of an optional <pose> child."""
    p = el.find("pose")
    v = np.zeros(6) if p is None or not (p.text or "").strip() else np.array(p.text.split(), float)
    if np.any(np.abs(v[3:5]) > 1e-9):
        raise ValueError("only planar poses (roll = pitch = 0) are supported")
    return v


def _compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pose b expressed in frame a -> world (planar)."""
    c, s = np.cos(a[5]), np.sin(a[5])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2], 0.0, 0.0, a[5] + b[5]])


@dataclass
class World:
    boxes: List[Box]

    @classmethod
    def from_sdf(cls, path, plane_z: float) -> "World":
        """Every <collision> box of every model that spans the scan plane
        ``plane_z`` (floors and flat markers fall outside it).  Non-box
        geometries are not supported and raise."""
        boxes = []
        for model in ET.parse(path).getroot().iter("model"):
            pm = _pose(model)
            for link in model.findall("link"):
                pl = _compose(pm, _pose(link))
                for col in link.findall("collision"):
                    pc = _compose(pl, _pose(col))
                    geom = col.find("geometry")
                    box = geom.find("box")
                    if box is None:
                        raise ValueError(f"model {model.get('name')!r}: only box collisions are supported")
                    sx, sy, sz = (float(v) for v in box.find("size").text.split())
                    if not (pc[2] - sz / 2 <= plane_z <= pc[2] + sz / 2):
                        continue
                    boxes.append(Box((pc[0], pc[1]), (sx / 2, sy / 2), float(pc[5])))
        return cls(boxes)

    def raycast(self, pos, phis: np.ndarray, range_max: float) -> np.ndarray:
        """Range along each world-frame direction ``phis`` from ``pos``; +inf
        where nothing is hit within ``range_max`` (LaserScan convention)."""
        r = np.full(len(phis), np.inf)
        for b in self.boxes:
            o = b.to_local(pos)[0]
            d = np.stack([np.cos(phis - b.yaw), np.sin(phis - b.yaw)], 1)
            inv = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
            t1 = (-np.asarray(b.half) - o) * inv
            t2 = (np.asarray(b.half) - o) * inv
            tn, tf = np.minimum(t1, t2).max(1), np.maximum(t1, t2).min(1)
            t = np.where(tn > 0, tn, tf)                 # from inside a box: its exit
            ok = (tn <= tf) & (t > 0)
            r = np.where(ok, np.minimum(r, t), r)
        return np.where(r < range_max, r, np.inf)

    def distance(self, xy: np.ndarray) -> np.ndarray:
        """Signed distance to the nearest box."""
        return np.min([b.distance(xy) for b in self.boxes], axis=0) if self.boxes else np.full(len(np.atleast_2d(xy)), np.inf)
