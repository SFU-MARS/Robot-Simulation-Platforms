"""
ROS-free closed loop, the stand-in for one Gazebo run.

Every control period (``profile.ctrl_dt``): scan the true world at the
current pose, update the accumulated map, ask the nominal controller for a
control, pass it through the optional safety hook, and hold it while the
plant moves: Dubins 3D (a unicycle at the profile's constant speed,
integrated exactly as circular arcs; control w) or Dubins 4D
(``profile.model``, exact, speed limits included; state (x, y, theta, v),
control (a, w)).  The footprint is checked against the true world at every
sub-step.

The safety hook is where the FaST-BRT filter plugs in:
``safety(x, u_nom, grid) -> u``, either one control for the period or an
array of zero-order-hold controls splitting it evenly (Dubins 3D: rates (k,);
Dubins 4D: (2,) or (k, 2)).  A RuntimeError from the
hook ends the run as "init_failed" (no certificate at the start) or
"filter_error" (later); a NotImplementedError ends it as "unsupported".  Without a
hook the robot runs the nominal controller only.
"""

from __future__ import annotations

import copy

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

from ..mapping import OccupancyGrid, extract_obstacles
from .world import World

__all__ = ["unicycle_step", "Episode", "run_episode"]


def unicycle_step(x: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
    """Exact step of (x, y, theta) for (v, w) held over ``dt``."""
    px, py, th = x
    if abs(w) < 1e-9:
        return np.array([px + v * dt * np.cos(th), py + v * dt * np.sin(th), th])
    th1 = th + w * dt
    r = v / w
    return np.array([px + r * (np.sin(th1) - np.sin(th)), py + r * (np.cos(th) - np.cos(th1)),
                     (th1 + np.pi) % (2 * np.pi) - np.pi])


@dataclass
class Episode:
    status: str                              # reached | collision | timeout | init_failed | filter_error | unsupported
    t: float                                 # s, time at the end
    traj: np.ndarray                         # (N, 3) states at every sub-step
    cmds: np.ndarray                         # per control period: (K, 2) (w_nom, w); 4D (K, 4) (a_nom, w_nom, a, w)
    grid: OccupancyGrid                      # map at the end
    snapshots: List[tuple] = field(default_factory=list)    # (t, pose, copy of the map)
    error: str = ""                                          # message when the filter stopped the run
    events: List = field(default_factory=list)              # ObstacleEvent per period (with a tracker)


def run_episode(profile, world: World, x0, goal, nominal, safety: Optional[Callable] = None,
                t_max: float = 60.0, goal_radius: float = 0.1, n_sub: int = 10,
                snapshot_every: Optional[float] = None, noise_std: float = 0.0, seed: int = 0,
                grid: Optional[OccupancyGrid] = None, tracker=None) -> Episode:
    """``nominal.control(x, goal, grid) -> u``; ``noise_std``: Gaussian range noise (m);
    ``grid``: the (empty) map to fill, default the profile's map; ``tracker``: an
    ``ObstacleTracker`` fed with the obstacle layer every period (events in ``Episode.events``).
    Dubins 4D: ``x0`` is (x, y, theta, v), or (x, y, theta) standing still."""
    profile.require_ready()
    dt = profile.ctrl_dt
    four = profile.dynamics == "dubins4d"
    plant = profile.model
    if four:
        if len(x0) == 3:
            x0 = (*x0, 0.0)
        step = plant.step
    else:
        v = profile.phys["v"]
        step = lambda x, u, h: unicycle_step(x, v, float(np.ravel(u)[0]), h)
    s = profile.sensor
    angles = np.arange(s.n_rays) * s.fov / s.n_rays
    rng = np.random.default_rng(seed)
    grid = OccupancyGrid.from_profile(profile) if grid is None else grid
    x = np.array(x0, float)
    traj, cmds, snaps = [x.copy()], [], []
    t = 0.0
    error = ""
    events = []
    every = None if snapshot_every is None else max(1, int(round(snapshot_every / dt)))
    status = "timeout"
    for k in range(int(round(t_max / dt))):
        xs = profile.sensor_pose(x)                  # the LiDAR is not at the wheel axle (profile sensor.offset)
        ranges = world.raycast(xs[:2], xs[2] + angles, s.range)
        if noise_std > 0:
            ranges = ranges + rng.normal(0.0, noise_std, ranges.shape)
        grid.integrate_scan(xs, angles, ranges, s.range, s.range_min)
        if tracker is not None:
            events += tracker.update(extract_obstacles(grid), t, grid.res)[1]
        if every is not None and k % every == 0:
            snaps.append((t, x.copy(), copy.deepcopy(grid)))
        u_nom = nominal.control(x, goal, grid)
        try:
            us = u_nom if safety is None else safety(x, u_nom, grid)
        except NotImplementedError as e:             # a case the filter does not support (subclass of RuntimeError)
            status, error = "unsupported", str(e)
            break
        except RuntimeError as e:                    # no certificate at x0, or a filter failure later
            status, error = ("init_failed" if k == 0 else "filter_error"), str(e)
            break
        us = np.asarray(us, float).reshape(-1, plant.n_control)
        us = np.clip(us, plant.u_low, plant.u_high)
        cmds.append((*np.ravel(u_nom), *us[0]) if four else (u_nom, float(us[0, 0])))
        m = max(1, n_sub // len(us))                 # plant sub-steps per held control
        for u in us:
            for _ in range(m):
                h = dt / (len(us) * m)
                x = step(x, u, h)
                t += h
                traj.append(x.copy())
                if world.distance(x[None, :2])[0] <= profile.footprint_radius:
                    status = "collision"
                elif np.linalg.norm(x[:2] - np.asarray(goal)) <= goal_radius:
                    status = "reached"
                if status != "timeout":
                    break
            if status != "timeout":
                break
        if status != "timeout":
            break
    snaps.append((t, x.copy(), copy.deepcopy(grid)))
    return Episode(status, t, np.array(traj), np.array(cmds), grid, snaps, error, events)
