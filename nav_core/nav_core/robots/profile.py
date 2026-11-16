"""
Robot profiles (robots/<name>.yaml): the single source of the per-robot
parameters -- dynamics name, physical limits, footprint, sensor, map, world.

Units.  FaST-BRT's Dubins 3D model is nondimensional (v = 1, |w| <= 1, turning
radius 1).  For a robot with speed v and turn-rate bound w_max, one model
length unit is v / w_max metres and one model time unit is 1 / w_max seconds.
Other dynamics use identity units until their models fix a scaling.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

__all__ = ["SensorSpec", "MapSpec", "Units", "RobotProfile", "load_profile", "ROBOTS_DIR"]

ROBOTS_DIR = Path(__file__).resolve().parents[3] / "robots"


@dataclass(frozen=True)
class SensorSpec:
    type: Optional[str]
    range: Optional[float]               # m
    fov: Optional[float]                 # rad
    n_rays: Optional[int]
    rate_hz: Optional[float]
    range_min: float = 0.0               # m
    height: Optional[float] = None       # m, scan plane above the floor


@dataclass(frozen=True)
class MapSpec:
    origin: Optional[Tuple[float, float]]    # m, lower-left corner
    size: Optional[Tuple[float, float]]      # m
    resolution: Optional[float]              # m / cell


@dataclass(frozen=True)
class Units:
    """Physical <-> model scaling: x_model = x_phys / length, t_model = t_phys / time."""
    length: float = 1.0
    time: float = 1.0

    def pos_to_model(self, xy):
        return np.asarray(xy, float) / self.length

    def pos_to_phys(self, xy):
        return np.asarray(xy, float) * self.length

    def rate_to_model(self, w):
        """Angular rate (rad / s) -> rad per model time unit."""
        return np.asarray(w, float) * self.time

    def rate_to_phys(self, w):
        return np.asarray(w, float) / self.time


@dataclass(frozen=True)
class RobotProfile:
    name: str
    status: str                          # "ready" or "placeholder"
    dynamics: str                        # model name in FaST-BRT
    phys: dict
    footprint_radius: Optional[float]    # m
    sensor: SensorSpec
    ctrl_dt: Optional[float]             # s
    map: MapSpec
    world: Optional[str]                 # path relative to the repo root
    base_value: Optional[str]

    @property
    def units(self) -> Units:
        if self.dynamics == "dubins3d":
            return Units(length=self.phys["v"] / self.phys["w_max"], time=1.0 / self.phys["w_max"])
        return Units()

    def require_ready(self) -> "RobotProfile":
        if self.status != "ready":
            raise RuntimeError(f"robot profile {self.name!r} is a {self.status}; fill robots/{self.name}.yaml first")
        return self

    def check_limits(self) -> None:
        """Raise if the dynamics parameters exceed the robot's actuation.
        Dubins 3D on a differential drive: the outer wheel runs at
        v + w_max * track_width / 2, so v and w_max cannot both be at their
        separate maxima.  Other dynamics: no check until their models define one."""
        p = self.phys
        if self.dynamics == "dubins3d" and "track_width" in p and "wheel_v_max" in p:
            wheel = p["v"] + p["w_max"] * p["track_width"] / 2
            if wheel > p["wheel_v_max"] + 1e-12:
                raise ValueError(f"{self.name}: v + w_max * track_width / 2 = {wheel:.3f} m/s exceeds "
                                 f"wheel_v_max = {p['wheel_v_max']} m/s")


def load_profile(name_or_path) -> RobotProfile:
    """``name_or_path``: a robot name (robots/<name>.yaml) or a YAML path."""
    p = Path(name_or_path)
    if p.suffix not in (".yaml", ".yml"):
        p = ROBOTS_DIR / f"{name_or_path}.yaml"
    d = yaml.safe_load(p.read_text())
    m = d["map"]
    profile = RobotProfile(
        name=d["name"], status=d["status"], dynamics=d["dynamics"], phys=dict(d.get("phys") or {}),
        footprint_radius=d["footprint_radius"], sensor=SensorSpec(**d["sensor"]), ctrl_dt=d["ctrl_dt"],
        map=MapSpec(origin=None if m["origin"] is None else tuple(m["origin"]),
                    size=None if m["size"] is None else tuple(m["size"]), resolution=m["resolution"]),
        world=d.get("world"), base_value=d.get("base_value"))
    if profile.status == "ready":
        profile.check_limits()
    return profile
