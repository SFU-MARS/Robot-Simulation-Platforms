"""
Witnesses (backup plans) for the constant-speed Dubins 3D model, in model units
(v = 1, |w| <= 1, turning radius 1).

A witness from a state is a zero-order-hold control sequence over the horizon
that keeps the scene margin above the clearance and ends on a free loiter
circle (turning on the spot forever).  The circle is free forever and the map
only gains free space, so a committed witness stays valid; the backup filter
(``backup_filter.py``) executes it whenever the nominal step has no witness of
its own.

The search rolls out all candidates as one batch, in priority order: the HJ
control of the scene (``scene.hj_control``, closed loop), then hard left,
hard right, straight, then straight for k steps and a hard turn
(k = 5, 10, ...).  Rollouts are exact arcs (``arc_step``); the margin is
sampled after every step and on 72 points of the loiter circle, so the
clearance must cover the distance to the nearest sample (``witness_slack``).

Ported from Hu-Hanyang/FaST-BRT (``fast_brt.certify.witness.find_witness``,
dives only).

``scene`` is any object with ``margin(xy) -> (N,)`` (signed distance to the
modelled obstacles, positive = free, 1-Lipschitz), ``hj_control(states) ->
(N,)`` turn rates, ``clearance`` and ``T`` (horizon).
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

__all__ = ["N_CIRCLE", "arc_step", "witness_slack", "candidate_controls", "loiter_sides", "find_witness",
           "witness_slack_4d", "find_witness_4d"]

N_CIRCLE = 72          # samples on a loiter circle
R_LOITER = 1.0         # loiter radius = turning radius in model units


def arc_step(z: np.ndarray, w: np.ndarray, dt: float) -> None:
    """Exact in-place step of states z (N, 3) at unit speed, turn rates w (N,) held over dt."""
    th = z[:, 2]
    straight = np.abs(w) < 1e-9
    r = 1.0 / np.where(straight, 1.0, w)
    th1 = th + w * dt
    z[:, 0] += np.where(straight, dt * np.cos(th), r * (np.sin(th1) - np.sin(th)))
    z[:, 1] += np.where(straight, dt * np.sin(th), r * (np.cos(th) - np.cos(th1)))
    z[:, 2] = np.mod(th1, 2.0 * np.pi)


def witness_slack(dt: float, R: float = R_LOITER, n_circle: int = N_CIRCLE) -> float:
    """Largest distance from a point of the continuous witness to the nearest sample: dt / 2 along
    an arc, 2 R sin(pi / (2 n)) on the loiter circle.  For a 1-Lipschitz margin, sampled margin >
    clearance >= slack implies a collision-free witness."""
    return max(dt / 2.0, 2.0 * R * np.sin(np.pi / (2 * n_circle)))


def candidate_controls(K: int) -> np.ndarray:
    """Open-loop candidates (C, K) in priority order: left, right, straight, then straight-then-turn."""
    rows = [np.full(K, w) for w in (1.0, -1.0, 0.0)]
    for ks in range(5, K, 5):
        for w in (1.0, -1.0):
            rows.append(np.r_[np.zeros(ks), np.full(K - ks, w)])
    return np.array(rows)


def loiter_sides(states: np.ndarray, scene, R: float = R_LOITER) -> np.ndarray:
    """+1 where the left loiter circle is free (margin > clearance on all samples), else -1 where the
    right one is, else 0."""
    states = np.atleast_2d(states)
    if len(states) == 0:
        return np.zeros(0)
    ang = np.linspace(0, 2 * np.pi, N_CIRCLE, endpoint=False)
    pts = []
    for side in (1.0, -1.0):
        cx = states[:, 0] - side * R * np.sin(states[:, 2])
        cy = states[:, 1] + side * R * np.cos(states[:, 2])
        pts.append(np.stack([cx[:, None] + R * np.cos(ang), cy[:, None] + R * np.sin(ang)], -1).reshape(-1, 2))
    margin = scene.margin(np.vstack(pts)).reshape(2, len(states), -1)
    left, right = (margin[0] > scene.clearance).all(1), (margin[1] > scene.clearance).all(1)
    return np.where(left, 1.0, np.where(right, -1.0, 0.0))


def find_witness(x0, scene, dt: float, use_hj: bool = True, info: Optional[dict] = None
                 ) -> Optional[Tuple[np.ndarray, float]]:
    """First candidate (priority order, see the module docstring) that keeps margin > clearance and
    ends on a free loiter circle.  Returns (controls (K,), loiter direction +1 / -1) or None.  ``info``
    (a dict) gets "candidates" and "index" (of the witness in priority order, 0 = the HJ one with use_hj;
    None without a witness)."""
    slack = witness_slack(dt)
    if scene.clearance < slack:
        raise ValueError(f"clearance {scene.clearance:g} < witness sampling slack {slack:.4g} (dt = {dt:g})")
    K = int(round(scene.T / dt))
    W = candidate_controls(K)
    C = len(W) + int(use_hj)
    x0 = np.asarray(x0, float)
    z = np.repeat(x0[None], C, axis=0)
    controls = np.empty((C, K))
    margin = np.full(C, scene.margin(x0[None, :2])[0])
    for k in range(K):
        w = W[:, k]
        if use_hj:
            w = np.concatenate([np.asarray(scene.hj_control(z[:1]), float).reshape(1), w])
        controls[:, k] = w
        arc_step(z, w, dt)
        margin = np.minimum(margin, scene.margin(z[:, :2]))
    ok = np.flatnonzero(margin > scene.clearance)
    if info is not None:
        info["candidates"], info["index"] = C, None
    if ok.size == 0:
        return None
    sides = loiter_sides(z[ok], scene)
    hit = np.flatnonzero(sides != 0.0)
    if hit.size == 0:
        return None
    if info is not None:
        info["index"] = int(ok[hit[0]])
    return controls[ok[hit[0]]], float(sides[hit[0]])


# ---- Dubins 4D: the backup plan is to brake to a stop ----

W_BRAKE = (1.0, -1.0, 0.0, 0.5, -0.5)   # turn rates (times w_max) of the braking candidates, after the HJ one


def witness_slack_4d(dt: float, v_bound: float = 1.0) -> float:
    """Largest distance from a point of the continuous witness to the nearest sample: |v| dt / 2 (no loiter
    circle; a stopped robot does not move)."""
    return v_bound * dt / 2.0


def find_witness_4d(x0, scene, plant, dt: float, use_hj: bool = True, info: Optional[dict] = None
                    ) -> Optional[Tuple[np.ndarray, None]]:
    """Dubins 4D (model units, state (x, y, theta, v), ``plant`` a ``Dubins4D`` in model units): the first
    candidate that brakes to a stop, keeping margin > clearance on the way.  Every candidate brakes as hard as
    needed, a = clip(-v / dt, -a_max, a_max), so the speed reaches 0 exactly at the end of a step; they differ in
    the turn rate held while braking: the HJ control's (closed loop), then ``W_BRAKE``.  Stopped is safe forever
    (static obstacles, the map only gains free space).  Returns (controls (K, 2), None) or None.  ``info`` as
    for ``find_witness``."""
    slack = witness_slack_4d(dt, max(abs(plant.v_min), abs(plant.v_max)))
    if scene.clearance < slack:
        raise ValueError(f"clearance {scene.clearance:g} < witness sampling slack {slack:.4g} (dt = {dt:g})")
    K = int(round(scene.T / dt))
    W = np.array(W_BRAKE) * plant.w_max
    C = len(W) + int(use_hj)
    z = np.repeat(np.asarray(x0, float)[None], C, axis=0)
    controls = np.zeros((C, K, 2))
    margin = np.full(C, scene.margin(z[:1, :2])[0])
    for k in range(K):
        moving = np.abs(z[:, 3]) > 1e-12
        if not moving.any():
            break
        w = W if not use_hj else np.concatenate([[float(scene.hj_control(z[:1])[0, 1])], W])
        a = np.clip(-z[:, 3] / dt, -plant.a_max, plant.a_max)
        u = np.stack([np.where(moving, a, 0.0), np.where(moving, w, 0.0)], 1)
        controls[:, k] = u
        z = plant.step_batch(z, u, dt)
        z[np.abs(z[:, 3]) < 1e-12, 3] = 0.0
        margin = np.minimum(margin, scene.margin(z[:, :2]))
    ok = np.flatnonzero((margin > scene.clearance) & (z[:, 3] == 0.0))
    if info is not None:
        info["candidates"], info["index"] = C, (int(ok[0]) if ok.size else None)
    if ok.size == 0:
        return None
    return controls[ok[0]], None
