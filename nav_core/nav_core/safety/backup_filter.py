"""
Backup filter (Dubins 3D) as a ``nav_core.sim`` safety hook.

Each period the filter builds the scene (``scene.py``: the region from the
accumulated map, one primitive per edge, V0) and accepts the nominal turn
rate only if a witness (backup plan, ``witness.py``) exists from the next
state: a rollout that keeps the clearance and ends on a free loiter circle.
Otherwise it executes the previously committed witness.  Ported from the
witness gatekeeper of Hu-Hanyang/FaST-BRT.

The modelled obstacle for the witness check (``Scene.margin``) is the
complement of the region together with the primitive rectangles, so it covers
every detected obstacle cell (grown by the footprint) completely, not just
its outline.  Safety rests on this geometry only; the HJ value of the
primitives proposes the first candidate of the search (``Scene.hj_control``,
``use_hj``).  See ``scene.py`` for ``v0``, ``unknown``, ``primitives``,
``placement`` and ``clearance``.
"""

from __future__ import annotations

import numpy as np

from .scene import SceneFilter
from .witness import arc_step, find_witness, witness_slack

__all__ = ["BackupFilter"]


class BackupFilter(SceneFilter):
    def __init__(self, profile, v0=None, use_hj: bool = True, **kw):
        super().__init__(profile, v0, **kw)
        if self.clearance < witness_slack(self.sub_dt):
            raise ValueError(f"clearance {self.clearance:g} (model units) < witness sampling slack "
                             f"{witness_slack(self.sub_dt):.4g}")
        self.use_hj = use_hj
        self.K = int(round(self.horizon / self.sub_dt))
        self.plan = None                               # committed witness, padded to K steps with its loiter turn
        self.loiter = 0.0
        self.started = False

    def _commit(self, found) -> None:
        plan, self.loiter = found
        if plan.size < self.K:
            plan = np.concatenate([plan, np.full(self.K - plan.size, self.loiter)])
        self.plan = plan

    def _start(self, model, xm) -> None:
        """Commit a witness from the first state; RuntimeError if there is none (the robot does not start)."""
        if not self.started:
            found = find_witness(xm, model, self.sub_dt, self.use_hj)
            if found is None:
                raise RuntimeError("no witness at the start")
            self._commit(found)
            self.started = True

    def _gate(self, model, xm, ws: np.ndarray):
        """Turn rates (model units) for this period: ``ws`` if it keeps the clearance and a witness exists from
        where it ends (the witness becomes the plan), else the committed plan.  Returns (rates, accepted)."""
        z = xm[None].copy()
        seg_margin = np.inf
        for w in ws:
            arc_step(z, np.array([w]), self.sub_dt)
            seg_margin = min(seg_margin, model.margin(z[:, :2])[0])
        found = find_witness(z[0], model, self.sub_dt, self.use_hj) if seg_margin > model.clearance else None
        if found is not None:
            self._commit(found)
            return np.asarray(ws, float), True
        plan = self.plan[:self.n_sub]                  # execute the committed plan, keep loitering after it
        self.plan = np.concatenate([self.plan[self.n_sub:], np.full(self.n_sub, self.loiter)])
        return np.asarray(plan, float), False

    def __call__(self, x, w_nom: float, grid) -> np.ndarray:
        """ZOH turn rates (rad / s) for the ``n_sub`` sub-steps of this period."""
        model = self.scene(x, grid)
        self.model = model
        xm = self.to_model(x)
        self._start(model, xm)
        wm = float(np.clip(self.units.rate_to_model(w_nom), -1.0, 1.0))
        ws, accepted = self._gate(model, xm, np.full(self.n_sub, wm))
        self.steps += 1
        self.interventions += int(not accepted)
        return self.units.rate_to_phys(ws)
