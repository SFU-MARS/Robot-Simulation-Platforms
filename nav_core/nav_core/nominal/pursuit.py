"""Pure pursuit to a goal point: saturated proportional heading control."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["Pursuit"]


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

