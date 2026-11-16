"""
Nominal (performance) controllers in physical units; no safety guarantee and
no knowledge of obstacles.  The safety filter (FaST-BRT) consumes their output.
New controllers (generic MPPI on FaST-BRT dynamics) go here.
"""

from .pursuit import Pursuit, make_nominal

__all__ = ["Pursuit", "make_nominal"]
