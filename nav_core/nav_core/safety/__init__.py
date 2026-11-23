"""
Safety filters for ``nav_core.sim`` (and the ROS 2 node).  The backup filter
and its witness search are here; the HJ value of the primitives comes from an
offline V0 read with ``fast_brt.query`` (SFU-MARS/fast-brt, numpy only).
"""

from .backup_filter import BackupFilter, Scene, load_v0, place_faces
from .witness import find_witness

__all__ = ["BackupFilter", "Scene", "find_witness", "load_v0", "place_faces"]
