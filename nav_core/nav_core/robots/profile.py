"""
Robot profiles (robots/<name>.yaml): the single source of the per-robot
parameters -- dynamics name, physical limits, footprint, sensor, map, world.

Units.  fast-brt's models are nondimensional.  Dubins 3D: v = 1, |w| <= 1
(turning radius 1); for a robot with speed v and turn-rate bound w_max, one
model length unit is v / w_max metres and one model time unit is 1 / w_max
seconds.  Dubins 4D: |w| <= 1 and v_max = 1, so the same with v = v_max; the
other limits follow (v_min v / ..., a_max time^2 / length).  Other dynamics use
identity units until their models fix a scaling.

``profile.model`` is the plant (``dynamics.py``): Dubins3D (x, y, theta) or
Dubins4D (x, y, theta, v), in physical units.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

from .dynamics import Dubins3D, Dubins4D

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
    offset: Tuple[float, float] = (0.0, 0.0)   # m, sensor position in the robot frame (x forward, y left)


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

    def speed_to_model(self, v):
        """Speed (m / s) -> model length per model time."""
        return np.asarray(v, float) * self.time / self.length

    def speed_to_phys(self, v):
        return np.asarray(v, float) * self.length / self.time

    def accel_to_model(self, a):
        """Acceleration (m / s^2) -> model units."""
        return np.asarray(a, float) * self.time ** 2 / self.length

    def accel_to_phys(self, a):
        return np.asarray(a, float) * self.length / self.time ** 2


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
    base_value: Optional[str]            # offline V0 file, absolute or relative to ``root``
    root: Optional[Path] = None          # the folder that holds robots/ (the repo root): relative paths start here

    def sensor_pose(self, x) -> np.ndarray:
        """Pose (x, y, theta) of the sensor for the robot pose x: the robot pose moved by ``sensor.offset``, which
        is given in the robot frame. A scan is cast from, and integrated at, this pose."""
        x = np.asarray(x, float)
        ox, oy = self.sensor.offset
        c, s = np.cos(x[2]), np.sin(x[2])
        return np.array([x[0] + c * ox - s * oy, x[1] + s * ox + c * oy, x[2]])

    @property
    def units(self) -> Units:
        if self.dynamics == "dubins3d":
            return Units(length=self.phys["v"] / self.phys["w_max"], time=1.0 / self.phys["w_max"])
        if self.dynamics == "dubins4d":
            return Units(length=self.phys["v_max"] / self.phys["w_max"], time=1.0 / self.phys["w_max"])
        return Units()

    @property
    def model(self):
        """The plant in physical units (``dynamics.Dubins3D`` or ``Dubins4D``)."""
        p = self.phys
        if self.dynamics == "dubins3d":
            return Dubins3D(p["v"], p["w_max"])
        if self.dynamics == "dubins4d":
            return Dubins4D(p["v_min"], p["v_max"], p["a_max"], p["w_max"])
        raise NotImplementedError(f"no plant model for dynamics {self.dynamics!r}")

    def require_ready(self) -> "RobotProfile":
        if self.status != "ready":
            raise RuntimeError(f"robot profile {self.name!r} is a {self.status}; fill robots/{self.name}.yaml first")
        return self

    def check_limits(self) -> None:
        """Raise if the dynamics parameters exceed the robot's actuation.
        On a differential drive the outer wheel runs at v + w_max * track_width / 2
        (Dubins 3D: v; Dubins 4D: the larger of |v_min| and v_max), so the speed
        and w_max cannot both be at their separate maxima.  Other dynamics: no
        check until their models define one."""
        p = self.phys
        speed = {"dubins3d": lambda: p["v"], "dubins4d": lambda: max(abs(p["v_min"]), abs(p["v_max"]))}
        if self.dynamics in speed and "track_width" in p and "wheel_v_max" in p:
            wheel = speed[self.dynamics]() + p["w_max"] * p["track_width"] / 2
            if wheel > p["wheel_v_max"] + 1e-12:
                raise ValueError(f"{self.name}: speed + w_max * track_width / 2 = {wheel:.3f} m/s exceeds "
                                 f"wheel_v_max = {p['wheel_v_max']} m/s")


def _sensor(d: dict) -> SensorSpec:
    d = dict(d)
    if d.get("offset") is not None:
        d["offset"] = tuple(float(v) for v in d["offset"])
    return SensorSpec(**d)


def load_profile(name_or_path) -> RobotProfile:
    """``name_or_path``: a robot name (robots/<name>.yaml) or a YAML path."""
    p = Path(name_or_path)
    if p.suffix not in (".yaml", ".yml"):
        p = ROBOTS_DIR / f"{name_or_path}.yaml"
    d = yaml.safe_load(p.read_text())
    m = d["map"]
    profile = RobotProfile(
        name=d["name"], status=d["status"], dynamics=d["dynamics"], phys=dict(d.get("phys") or {}),
        footprint_radius=d["footprint_radius"], sensor=_sensor(d["sensor"]), ctrl_dt=d["ctrl_dt"],
        map=MapSpec(origin=None if m["origin"] is None else tuple(m["origin"]),
                    size=None if m["size"] is None else tuple(m["size"]), resolution=m["resolution"]),
        world=d.get("world"), base_value=d.get("base_value"), root=p.resolve().parents[1])
    if profile.status == "ready":
        profile.check_limits()
    return profile
