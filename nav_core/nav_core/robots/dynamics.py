"""
Plant models of the robots in physical units, exact steps for a control held over a time step.

  Dubins3D -- (x, y, theta), constant speed v, control w (turn rate, |w| <= w_max): circular arcs.
  Dubins4D -- (x, y, theta, v), control (a, w): acceleration |a| <= a_max and turn rate |w| <= w_max, speed limits
              v_min <= v <= v_max in the dynamics (the acceleration stops at a limit), as HJ_local_update's
              dubins_4d. The pose comes first, so x[:3] is the pose for both robots. fast-brt's 4D V0 orders the
              state (x, y, v, theta); the safety filters convert.

The 4D step is exact: v is linear in time until it reaches a limit and constant after it, theta is linear, and
the position is the closed-form integral of v (cos theta, sin theta) over each piece.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["Dubins3D", "Dubins4D", "arc_integral"]


def arc_integral(th0: np.ndarray, v0: np.ndarray, a: np.ndarray, w: np.ndarray, h: np.ndarray):
    """(dx, dy) = integral over [0, h] of (v0 + a t) (cos, sin)(th0 + w t) dt, elementwise."""
    th0, v0, a, w, h = np.broadcast_arrays(*(np.asarray(q, float) for q in (th0, v0, a, w, h)))
    straight = np.abs(w) < 1e-6
    ws = np.where(straight, 1.0, w)
    th1 = th0 + w * h
    v1 = v0 + a * h
    c0, s0, c1, s1 = np.cos(th0), np.sin(th0), np.cos(th1), np.sin(th1)
    dx = (v1 * s1 - v0 * s0) / ws + a * (c1 - c0) / ws ** 2
    dy = (v0 * c0 - v1 * c1) / ws + a * (s1 - s0) / ws ** 2
    dist = v0 * h + 0.5 * a * h * h
    mid = th0 + 0.5 * w * h                                    # straight: |w h| < 1e-6 h, heading error negligible
    dx = np.where(straight, dist * np.cos(mid), dx)
    dy = np.where(straight, dist * np.sin(mid), dy)
    return dx, dy


@dataclass(frozen=True)
class Dubins3D:
    v: float
    w_max: float
    n_state: int = 3
    n_control: int = 1

    @property
    def u_low(self) -> np.ndarray:
        return np.array([-self.w_max])

    @property
    def u_high(self) -> np.ndarray:
        return np.array([self.w_max])

    def step(self, x, u, dt: float) -> np.ndarray:
        """Exact step of (x, y, theta) for the turn rate u (a number or a 1-vector) held over dt."""
        w = float(np.ravel(u)[0])
        return self.step_batch(np.asarray(x, float)[None], np.array([[w]]), dt)[0]

    def step_batch(self, z: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """Exact step of every row of z (N, 3) for turn rates u (N,) or (N, 1)."""
        w = np.reshape(u, (len(z),))
        dx, dy = arc_integral(z[:, 2], self.v, 0.0, w, dt)
        th1 = (z[:, 2] + w * dt + np.pi) % (2 * np.pi) - np.pi
        return np.stack([z[:, 0] + dx, z[:, 1] + dy, th1], 1)


@dataclass(frozen=True)
class Dubins4D:
    v_min: float
    v_max: float
    a_max: float
    w_max: float
    n_state: int = 4
    n_control: int = 2

    @property
    def u_low(self) -> np.ndarray:
        return np.array([-self.a_max, -self.w_max])

    @property
    def u_high(self) -> np.ndarray:
        return np.array([self.a_max, self.w_max])

    def step(self, x, u, dt: float) -> np.ndarray:
        """Exact step of (x, y, theta, v) for (a, w) held over dt."""
        return self.step_batch(np.asarray(x, float)[None], np.asarray(u, float).reshape(1, 2), dt)[0]

    def step_batch(self, z: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """Exact step of every row of z (N, 4) for controls u (N, 2): the speed follows a until it reaches v_min or
        v_max (then stays there), the heading turns at w."""
        th, v = z[:, 2], np.clip(z[:, 3], self.v_min, self.v_max)
        a, w = u[:, 0], u[:, 1]
        limit = np.where(a > 0, self.v_max, self.v_min)
        t1 = np.where(a != 0, (limit - v) / np.where(a != 0, a, 1.0), np.inf)
        t1 = np.clip(t1, 0.0, dt)                          # time spent accelerating
        dx1, dy1 = arc_integral(th, v, a, w, t1)
        v1 = v + a * t1
        th1 = th + w * t1
        dx2, dy2 = arc_integral(th1, v1, 0.0, w, dt - t1)   # at the limit (or no time left)
        th2 = (th + w * dt + np.pi) % (2 * np.pi) - np.pi
        return np.stack([z[:, 0] + dx1 + dx2, z[:, 1] + dy1 + dy2, th2, v1], 1)
