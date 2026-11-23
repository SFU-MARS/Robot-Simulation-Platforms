import math

import numpy as np
import pytest

from nav_core.robots import load_profile
from nav_core.safety import load_v0
from nav_core.safety.backup_filter import REPO_ROOT


@pytest.fixture(scope="session")
def v0():
    """The Burger's V0 (robots/burger.yaml base_value); not in git, so the closed-loop tests skip without it"""
    path = REPO_ROOT / load_profile("burger").base_value
    if not path.is_file():
        pytest.skip(f"no V0 file at {path} (python -m fast_brt export-v0 in fast-brt)")
    return load_v0(path)


@pytest.fixture(scope="session")
def fake_v0():
    """A made-up V0 = l_0 (as if the horizon were 0) on the grid of the recommended config: the placement
    and the geometry are right, the HJ control is not (dV/dtheta = 0)"""
    from fast_brt.query import PrimitiveV0, kernel
    n, nt = 241, 60
    xs = np.linspace(-6.0, 6.0, n)
    value = np.broadcast_to(kernel(xs[:, None], xs[None, :], 20.0, "linear")[..., None], (n, n, nt))
    return PrimitiveV0(value.astype(np.float32), [-6.0, -6.0, -math.pi], [6.0, 6.0, math.pi], [n, n, nt], [2],
                       20.0, "linear", 2.0, "dubins_3d", {"v": 1, "wMax": 1, "uMode": "max"})
