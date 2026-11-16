"""Pure pursuit to a goal point: saturated proportional heading control."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["Pursuit", "make_nominal"]


@dataclass
class Pursuit:
    """Dubins 3D (x, y, theta): returns the turn rate in rad / s."""
    w_max: float
    k_gain: float = 2.0

    def control(self, x, goal) -> float:
        bearing = np.arctan2(goal[1] - x[1], goal[0] - x[0])
        err = (bearing - x[2] + np.pi) % (2 * np.pi) - np.pi
        return float(np.clip(self.k_gain * err, -self.w_max, self.w_max))


def make_nominal(profile, kind: str = "pursuit"):
    profile.require_ready()
    if kind == "pursuit" and profile.dynamics == "dubins3d":
        return Pursuit(w_max=profile.phys["w_max"])
    raise NotImplementedError(f"nominal {kind!r} for dynamics {profile.dynamics!r}")
