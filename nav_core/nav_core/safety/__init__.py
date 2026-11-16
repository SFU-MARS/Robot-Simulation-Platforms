"""
Safety hooks that wrap FaST-BRT filters for ``nav_core.sim`` (and later the
ROS 2 node).  The filters themselves live in FaST-BRT; only unit conversion
and the map -> scene interface are here.  ``fast_brt`` is imported lazily.
"""

from .fastbrt_gatekeeper import FastBrtGatekeeper

__all__ = ["FastBrtGatekeeper"]
