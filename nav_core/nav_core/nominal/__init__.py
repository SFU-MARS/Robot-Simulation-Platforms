"""
Nominal (performance) controllers in physical units, ``control(x, goal, grid)
-> turn rate``; no safety guarantee.  The safety filter (FaST-BRT) consumes
their output.

  pursuit -- turn towards the goal; ignores the map.
  mppi    -- sampling MPC on the accumulated map (geodesic cost-to-go and
             obstacle distance); optimistic about unknown space.
"""

from .mppi import MPPI, CostMap
from .pursuit import Pursuit

__all__ = ["Pursuit", "MPPI", "CostMap", "make_nominal"]


def make_nominal(profile, kind: str = "pursuit", **kw):
    profile.require_ready()
    if profile.dynamics != "dubins3d":
        raise NotImplementedError(f"nominal {kind!r} for dynamics {profile.dynamics!r}")
    if kind == "pursuit":
        return Pursuit(w_max=profile.phys["w_max"], **kw)
    if kind == "mppi":
        return MPPI(profile.phys["v"], profile.phys["w_max"], profile.footprint_radius, **kw)
    raise ValueError(f"unknown nominal {kind!r}")
