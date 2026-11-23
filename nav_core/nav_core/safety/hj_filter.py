"""
HJ filters (Dubins 3D) on the primitive values, as ``nav_core.sim`` safety hooks.

Both compare the distance margin of the union of the primitives (``Scene.hj_margin``: each face's value in the
linear encoding divided by its Lipschitz constant sqrt(alpha), so a state with margin d is at least d, at the
same heading, from the BRT of every face; the same meaning for both encodings) with ``margin`` (m).

``HJFilter`` (filter mode ``hj``) is the classic HJ filter.  Each period it builds the primitives (``scene.py``,
no region: it uses the primitives only) and predicts the nominal turn rate over the period (exact arcs, ``n_sub``
sub-steps).  If the margin stays above ``margin`` at every predicted sub-step, the nominal rate is applied.
Otherwise the optimal control of the nearest primitive, w = w_max sign(dV/dtheta), is applied, re-read at every
predicted sub-step (closed loop).  It is not safe on its own: the union of the primitive values leaks at the
corners where faces meet (U > 0 at states that hit the corner), and the offline V0 underestimates the BRT (about
0.09 turning radii in front of a face with the default low V0).  It is the baseline that the local update and
the backup filter improve on.  With ``unknown="occupied"`` the frontier edges are primitives too.  It corrects
the leaking corners by default (``corners=True``, docs/13): without it, the union leaks by up to a turning radius
at concave corners and the filter collides there.

``robust=True`` adds to ``margin`` what the classic filter leaves out: the map's ``clearance`` (as the backup
filter) and the sampling gap between checked sub-steps.  Between two sub-steps dt apart the position moves at most
v dt and the heading w_max dt, so the margin changes by at most (v + v T w_max) dt / 2 at the midpoint
(|dV/dtheta| <= sqrt(alpha) v T: the same control from a turned heading gives the turned trajectory).

``HJBackupFilter`` (filter mode ``hj_backup``): the HJ filter proposes the turn rates of the period (nominal or
HJ evasion, as above), and the backup filter (``backup_filter.py``) applies them only if a witness exists from
where they end; otherwise it executes the committed witness.  The HJ filter shapes when and how the robot turns
away, the backup filter keeps it safe (also at leaking corners).  Its ``margin`` must be positive (default 2 cm,
about 0.2 turning radii beyond the BRT): states with U <= 0 never have a witness, so the backup filter turns away
first and with ``margin`` 0 the filter is the backup filter.  No corner correction by default (it does not change
its safety, the backup filter already provides it; ``corners=True`` turns it on).  See ``scene.py`` for ``v0``,
``unknown``,
``primitives`` and ``placement``.
"""

from __future__ import annotations

import numpy as np

from .backup_filter import BackupFilter
from .scene import SceneFilter
from .witness import arc_step

__all__ = ["HJBackupFilter", "HJFilter", "hj_rates"]


def hj_rates(model, xm, wm: float, n_sub: int, dt: float, margin: float = 0.0):
    """Turn rates (model units) of the HJ filter for one period from xm: the nominal ``wm`` held if the distance
    margin (model units) stays above ``margin`` on its predicted sub-steps, else the closed-loop HJ evasion.
    Returns (rates, evaded)."""
    z = xm[None].copy()
    for _ in range(n_sub):
        arc_step(z, np.array([wm]), dt)
        if model.hj_margin(z)[0] <= margin:
            break
    else:
        return np.full(n_sub, wm), False
    z = xm[None].copy()
    ws = np.empty(n_sub)
    for k in range(n_sub):
        ws[k] = model.hj_control(z)[0]
        arc_step(z, ws[k:k + 1], dt)
    return ws, True


def model_margin(f, margin: float, robust: bool) -> float:
    """``margin`` (m) of filter ``f`` in model units, plus the clearance and the sampling gap if ``robust``."""
    m = margin / f.units.length
    if robust:
        m += f.clearance + (1.0 + f.v0.horizon) * f.sub_dt / 2.0          # v = w_max = 1 in model units
    return m


class HJFilter(SceneFilter):
    uses_region = False

    def __init__(self, profile, v0=None, margin: float = 0.0, robust: bool = False, corners: bool = True, **kw):
        super().__init__(profile, v0, corners=corners, **kw)
        self.margin, self.robust = margin, robust
        self.margin_model = model_margin(self, margin, robust)

    def __call__(self, x, w_nom: float, grid) -> np.ndarray:
        """ZOH turn rates (rad / s) for the ``n_sub`` sub-steps of this period."""
        model = self.scene(x, grid)
        self.model = model
        wm = float(np.clip(self.units.rate_to_model(w_nom), -1.0, 1.0))
        ws, evaded = hj_rates(model, self.to_model(x), wm, self.n_sub, self.sub_dt, self.margin_model)
        self.steps += 1
        self.interventions += int(evaded)
        return self.units.rate_to_phys(ws)


class HJBackupFilter(BackupFilter):
    def __init__(self, profile, v0=None, margin: float = 0.02, robust: bool = False, **kw):
        super().__init__(profile, v0, **kw)
        self.margin, self.robust = margin, robust
        self.margin_model = model_margin(self, margin, robust)
        self.hj_interventions = 0                      # periods in which the HJ evasion replaced the nominal rate
        self.backup_interventions = 0                  # periods in which the committed witness was executed

    def __call__(self, x, w_nom: float, grid) -> np.ndarray:
        """ZOH turn rates (rad / s) for the ``n_sub`` sub-steps of this period."""
        model = self.scene(x, grid)
        self.model = model
        xm = self.to_model(x)
        self._start(model, xm)
        wm = float(np.clip(self.units.rate_to_model(w_nom), -1.0, 1.0))
        proposed, evaded = hj_rates(model, xm, wm, self.n_sub, self.sub_dt, self.margin_model)
        ws, accepted = self._gate(model, xm, proposed)
        self.steps += 1
        self.hj_interventions += int(evaded and accepted)
        self.backup_interventions += int(not accepted)
        self.interventions += int(evaded or not accepted)
        return self.units.rate_to_phys(ws)
