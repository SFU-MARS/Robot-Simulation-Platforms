"""
Nominal (performance) controllers in physical units, ``control(x, goal, grid)
-> turn rate`` (Dubins 3D) or ``-> (a, w)`` (Dubins 4D); no safety guarantee.  The safety filter (FaST-BRT) consumes
their output.

  pursuit -- turn towards the goal; ignores the map.
  mppi    -- sampling MPC on the accumulated map (geodesic cost-to-go and
             obstacle distance); optimistic about unknown space.
"""

from .mppi import MPPI, CostMap
from .pursuit import Pursuit, Pursuit4D

__all__ = ["Pursuit", "Pursuit4D", "MPPI", "CostMap", "make_nominal"]


def make_nominal(profile, kind: str = "pursuit", **kw):
    profile.require_ready()
    if profile.dynamics == "dubins4d":
        p = profile.phys
        if kind == "pursuit":
            return Pursuit4D(w_max=p["w_max"], a_max=p["a_max"], v_max=p["v_max"], **kw)
        if kind == "mppi":
            # the soft obstacle cost decays over the safety filter's clearance (1.5 map cells) plus 2 cm instead of
            # 3 cm: a robot that stops for the filter (the 4D backup) waits as long as its plan hugs an obstacle
            kw.setdefault("margin", 1.5 * profile.map.resolution + 0.02)
            return MPPI.for_model(profile.model, profile.footprint_radius, **kw)
        raise ValueError(f"unknown nominal {kind!r}")
    if profile.dynamics != "dubins3d":
        raise NotImplementedError(f"nominal {kind!r} for dynamics {profile.dynamics!r}")
    if kind == "pursuit":
        return Pursuit(w_max=profile.phys["w_max"], **kw)
    if kind == "mppi":
        return MPPI(profile.phys["v"], profile.phys["w_max"], profile.footprint_radius, **kw)
    raise ValueError(f"unknown nominal {kind!r}")
