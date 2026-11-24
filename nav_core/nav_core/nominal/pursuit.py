"""Pure pursuit to a goal point: saturated proportional heading control (Dubins 4D: and a speed profile)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["Pursuit", "Pursuit4D"]


@dataclass
class Pursuit:
    """Dubins 3D (x, y, theta): returns the turn rate in rad / s."""
    w_max: float
    k_gain: float = 2.0

    def control(self, x, goal, grid=None) -> float:
        """Ignores the map."""
        bearing = np.arctan2(goal[1] - x[1], goal[0] - x[0])
        err = (bearing - x[2] + np.pi) % (2 * np.pi) - np.pi
        return float(np.clip(self.k_gain * err, -self.w_max, self.w_max))


@dataclass
class Pursuit4D:
    """Dubins 4D (x, y, theta, v): returns (a, w) in m / s^2 and rad / s.  The turn rate as ``Pursuit``; the speed
    follows v_max cos(heading error) (0 when the goal is behind), slowed down within ``slow_radius`` (m) of the goal,
    with the acceleration k_speed (v_des - v), saturated.  Ignores the map."""
    w_max: float
    a_max: float
    v_max: float
    k_gain: float = 2.0
    k_speed: float = 2.0
    slow_radius: float = 0.6

    def control(self, x, goal, grid=None) -> np.ndarray:
        d = np.asarray(goal[:2], float) - np.asarray(x[:2], float)
        err = (np.arctan2(d[1], d[0]) - x[2] + np.pi) % (2 * np.pi) - np.pi
        w = float(np.clip(self.k_gain * err, -self.w_max, self.w_max))
        v_des = self.v_max * max(0.0, np.cos(err)) * min(1.0, float(np.hypot(*d)) / self.slow_radius)
        a = float(np.clip(self.k_speed * (v_des - x[3]), -self.a_max, self.a_max))
        return np.array([a, w])
