"""
Safety filters for ``nav_core.sim`` (and the ROS 2 node): ``backup`` (``BackupFilter``), ``hj``
(``HJFilter``) and ``hj_backup`` (``HJBackupFilter``), on a common scene (``scene.py``).  The HJ value of the primitives comes from an offline V0 read
with ``fast_brt.query`` (SFU-MARS/fast-brt, numpy only).
"""

from .backup_filter import BackupFilter
from .hj_filter import HJBackupFilter, HJFilter
from .scene import Scene, SceneFilter, load_v0, place_faces
from .witness import find_witness

__all__ = ["FILTERS", "BackupFilter", "HJBackupFilter", "HJFilter", "Scene", "SceneFilter", "find_witness", "load_v0",
           "make_filter", "place_faces"]

FILTERS = {"backup": BackupFilter, "hj": HJFilter, "hj_backup": HJBackupFilter}


def make_filter(profile, kind: str, v0=None, **kw):
    """Safety hook by name, ``kind`` in FILTERS, or None for "none"."""
    if kind == "none":
        return None
    if kind not in FILTERS:
        raise ValueError(f"filter must be 'none' or one of {list(FILTERS)}, got {kind!r}")
    return FILTERS[kind](profile, v0, **kw)
