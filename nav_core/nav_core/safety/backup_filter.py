"""
Backup filter (Dubins 3D and 4D) as a ``nav_core.sim`` safety hook.

Each period the filter builds the scene (``scene.py``: the region from the
accumulated map, one primitive per edge, V0) and accepts the nominal control
only if a witness (backup plan, ``witness.py``) exists from the next state: a
rollout that keeps the clearance and ends safe forever -- on a free loiter
circle (Dubins 3D, constant speed) or stopped (Dubins 4D: brake to v = 0).
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
from .witness import arc_step, find_witness, find_witness_4d, witness_slack, witness_slack_4d

__all__ = ["BackupFilter"]


class BackupFilter(SceneFilter):
    def __init__(self, profile, v0=None, use_hj: bool = True, **kw):
        super().__init__(profile, v0, **kw)
        slack = witness_slack_4d(self.sub_dt) if self.four else witness_slack(self.sub_dt)
        if self.clearance < slack:
            raise ValueError(f"clearance {self.clearance:g} (model units) < witness sampling slack {slack:.4g}")
        self.use_hj = use_hj
        self.K = int(round(self.horizon / self.sub_dt))
        self.plan = None                               # committed witness, padded to K steps with its loiter turn
        self.loiter = 0.0                              # (Dubins 4D: padded with "stay stopped", a = w = 0)
        self.started = False
        # search statistics: witness searches, their time, how many found one, how many with the HJ candidate (the
        # first one when use_hj), and the sum of the indices of the witnesses found (priority order)
        self.searches = self.searches_found = self.searches_hj = self.search_index_sum = 0
        self.search_seconds = 0.0

    def _find(self, z, model):
        import time
        info = {}
        start = time.perf_counter()
        if self.four:
            found = find_witness_4d(z, model, self.plant_model, self.sub_dt, self.use_hj, info)
        else:
            found = find_witness(z, model, self.sub_dt, self.use_hj, info)
        self.search_seconds += time.perf_counter() - start
        self.searches += 1
        if info.get("index") is not None:
            self.searches_found += 1
            self.search_index_sum += info["index"]
            self.searches_hj += int(self.use_hj and info["index"] == 0)
        return found

    def _commit(self, found) -> None:
        plan, loiter = found
        if self.four:
            if len(plan) < self.K:
                plan = np.concatenate([plan, np.zeros((self.K - len(plan), 2))])
            self.plan = plan
            return
        self.loiter = loiter
        if plan.size < self.K:
            plan = np.concatenate([plan, np.full(self.K - plan.size, self.loiter)])
        self.plan = plan

    def _start(self, model, xm) -> None:
        """Commit a witness from the first state; RuntimeError if there is none (the robot does not start)."""
        if not self.started:
            found = self._find(xm, model)
            if found is None:
                raise RuntimeError("no witness at the start")
            self._commit(found)
            self.started = True

    def _gate(self, model, xm, ws: np.ndarray):
        """Controls (model units) for this period: ``ws`` if it keeps the clearance and a witness exists from
        where it ends (the witness becomes the plan), else the committed plan.  Returns (controls, accepted):
        turn rates (n_sub,) for Dubins 3D, (n_sub, 2) for Dubins 4D."""
        z = xm[None].copy()
        seg_margin = np.inf
        for w in ws:
            if self.four:
                z = self.plant_model.step_batch(z, np.asarray(w, float)[None], self.sub_dt)
            else:
                arc_step(z, np.array([w]), self.sub_dt)
            seg_margin = min(seg_margin, model.margin(z[:, :2])[0])
        found = self._find(z[0], model) if seg_margin > model.clearance else None
        if found is not None:
            self._commit(found)
            return np.asarray(ws, float), True
        plan = self.plan[:self.n_sub]                  # execute the committed plan, keep loitering (stopped) after it
        pad = np.zeros((self.n_sub, 2)) if self.four else np.full(self.n_sub, self.loiter)
        self.plan = np.concatenate([self.plan[self.n_sub:], pad])
        return np.asarray(plan, float), False

    def __call__(self, x, w_nom, grid) -> np.ndarray:
        """ZOH controls for the ``n_sub`` sub-steps of this period: turn rates (rad / s), or (a, w) rows
        (m / s^2, rad / s) for Dubins 4D."""
        model = self.scene(x, grid)
        self.model = model
        xm = self.to_model(x)
        self._start(model, xm)
        if self.four:
            ws, accepted = self._gate(model, xm, np.tile(self.control_to_model(w_nom), (self.n_sub, 1)))
            self.steps += 1
            self.interventions += int(not accepted)
            return self.controls_to_phys(ws)
        wm = float(np.clip(self.units.rate_to_model(w_nom), -1.0, 1.0))
        ws, accepted = self._gate(model, xm, np.full(self.n_sub, wm))
        self.steps += 1
        self.interventions += int(not accepted)
        return self.units.rate_to_phys(ws)
