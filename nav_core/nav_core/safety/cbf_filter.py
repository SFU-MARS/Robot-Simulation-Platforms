"""
Discrete-time CBF filter on the HJ value (filter mode ``hj_cbf``), as a ``nav_core.sim`` safety hook.

The barrier is the distance margin of the primitives' union (``Scene.hj_margin``, model length units, lowered at the
corrected corners when ``corners`` is on) minus ``margin``: h(x) = margin_HJ(x) - margin. At every sub-step of the
period the filter predicts one sub-step (exact dynamics) for the nominal control and for a fixed grid of controls,
and keeps the ones that satisfy the discrete-time exponential CBF condition (hj_nav's FI condition)

    h(x+) >= (1 - gamma) h(x),            gamma = alpha * dt in (0, 1]

Among them it applies the one closest to the nominal control (the nominal control itself when it satisfies the
condition; distances with each control scaled by its bound). When none does, it applies the one that maximises h(x+):
the most cautious control the value suggests, so that the filter never gets stuck on the first-order scheme's small
drift (about 0.01 of the value per step on the robot's own greedy path, docs/16 section 16.7). Inside the tube
(h < 0) the condition asks h to recover by a share gamma of the violation every sub-step.

Compared with the classic ``hj`` filter (the nominal rate, or the bang-bang HJ control for the whole period), it changes
the nominal control as little as the condition allows, and gamma sets how fast it may approach the tube. Like ``hj`` it
builds no region and has no backup plan: its safety rests on the value. ``corners`` as for ``hj`` (on for Dubins 3D).

Controls: Dubins 3D, the turn rate on ``n_w`` values in [-1, 1]; Dubins 4D, (a, w) on ``n_a`` x ``n_w`` values; both
plus the nominal control's own values.
"""

from __future__ import annotations

import numpy as np

from .scene import SceneFilter
from .witness import arc_step

__all__ = ["HJCBFFilter", "cbf_controls"]


def cbf_controls(model, xm, um, n_sub: int, dt: float, gamma: float, margin: float = 0.0, plant=None,
                 n_w: int = 9, n_a: int = 5):
    """Controls (model units) of the CBF filter for one period from xm with the nominal control um ((1,) turn rate for
    Dubins 3D, (a, w) with ``plant``, the Dubins 4D model in model units). Returns (controls (n_sub, m), modified,
    infeasible sub-steps)."""
    um = np.atleast_1d(np.asarray(um, float))
    if plant is None:
        hi = np.array([1.0])
        grid = np.linspace(-1.0, 1.0, n_w)[:, None]
        cand = np.unique(np.vstack([grid, um[None]]), axis=0)
    else:
        hi = plant.u_high
        a_vals = np.unique(np.r_[np.linspace(-hi[0], hi[0], n_a), um[0]])
        w_vals = np.unique(np.r_[np.linspace(-hi[1], hi[1], n_w), um[1]])
        cand = np.array([[a, w] for a in a_vals for w in w_vals])
    dist = np.sum(((cand - um[None]) / hi[None]) ** 2, axis=1)
    order = np.argsort(dist, kind="stable")               # the nominal control first, then by distance to it
    cand, dist = cand[order], dist[order]
    z = np.asarray(xm, float).copy()
    out = np.empty((n_sub, len(um)))
    infeasible = 0
    for k in range(n_sub):
        h = float(model.hj_margin(z[None])[0]) - margin
        Z = np.repeat(z[None], len(cand), axis=0)
        if plant is None:
            arc_step(Z, cand[:, 0], dt)
        else:
            Z = plant.step_batch(Z, cand, dt)
        hn = model.hj_margin(Z) - margin
        ok = hn >= (1.0 - gamma) * h
        if ok.any():
            i = int(np.flatnonzero(ok)[0])                # closest to the nominal control among the feasible ones
        else:
            i = int(np.argmax(hn))
            infeasible += 1
        out[k] = cand[i]
        z = Z[i]
    modified = bool(np.any(np.abs(out - um[None]) > 1e-9))
    return out, modified, infeasible


class HJCBFFilter(SceneFilter):
    uses_region = False

    def __init__(self, profile, v0=None, gamma: float = 0.2, margin: float = 0.0, corners: "bool | None" = None,
                 n_w: int = 9, n_a: int = 5, **kw):
        if corners is None:                        # as hj: on for Dubins 3D (a 4D window takes 1-6 s, docs/15)
            corners = profile.dynamics == "dubins3d"
        super().__init__(profile, v0, corners=corners, **kw)
        if not 0.0 < gamma <= 1.0:
            raise ValueError(f"gamma (alpha * dt per sub-step) must be in (0, 1], got {gamma}")
        self.gamma, self.margin, self.n_w, self.n_a = gamma, margin, n_w, n_a
        self.margin_model = margin / self.units.length
        self.infeasible = 0                        # sub-steps where no control met the condition (max h applied)

    def __call__(self, x, u_nom, grid) -> np.ndarray:
        """ZOH controls for the ``n_sub`` sub-steps of this period: turn rates (rad / s), or (a, w) rows for 4D."""
        model = self.scene(x, grid)
        self.model = model
        us, modified, infeasible = cbf_controls(model, self.to_model(x), self.control_to_model(u_nom), self.n_sub,
                                                self.sub_dt, self.gamma, self.margin_model,
                                                self.plant_model if self.four else None, self.n_w, self.n_a)
        self.steps += 1
        self.interventions += int(modified)
        self.infeasible += infeasible
        return self.controls_to_phys(us)
